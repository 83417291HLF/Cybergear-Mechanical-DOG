#include "jump.h"
#include "leg.h"
#include <math.h>

static inline float clamp_f(float v, float min, float max)
{
    return (v < min) ? min : ((v > max) ? max : v);
}

static inline float smoothstep(float t)
{
    return t * t * (3.0f - 2.0f * t);
}

static JumpController g_jump = {
    .state = JUMP_IDLE,
    .timer = 0.0f,
    .params = {
        .crouch_height = 190.0f,   // 下蹲高度
        .lean_angle    = 18.0f,    // 前倾角度
        .jump_power    = 1.0f,     // 跳跃力度
        .push_distance = 30.0f,    // 蹬地距离
        .aerial_height = 185.0f,   // 腾空收腿高度
        .land_buffer   = 1.0f,     // 落地缓冲系数
    },
    .gains = {
        [JUMP_IDLE]         = {.kp = 12.0f, .kd = 0.8f},
        [JUMP_CROUCH]       = {.kp = 12.0f, .kd = 0.8f},
        [JUMP_LEAN_FORWARD] = {.kp = 12.0f, .kd = 0.8f},
        [JUMP_EXPLODE]      = {.kp = 12.0f, .kd = 0.8f},
        [JUMP_AERIAL]       = {.kp = 12.0f, .kd = 0.8f},
        [JUMP_LAND]         = {.kp = 12.0f, .kd = 0.8f},
        [JUMP_RECOVER]      = {.kp = 12.0f, .kd = 0.8f},
    }
};

static uint8_t btn_was_pressed = 0;

static void leg_control_with_gains(uint8_t leg_idx, float x, float y, JumpGains *g)
{
    float cx = -x;
    float cy = y;
    if (leg_idx >= 2) cx = -cx;

    float L_sq = cx * cx + cy * cy;
    float L = sqrtf(L_sq);
    L = clamp_f(L, fabsf(L1 - L2_EFF) + 5.0f, (L1 + L2_EFF) * 0.95f);

    float psi = asinf(clamp_f(-cx / L, -1.0f, 1.0f));
    float phi = acosf(clamp_f((L_sq + L1 * L1 - L2_EFF * L2_EFF) / (2.0f * L1 * L), -1.0f, 1.0f));

    float theta1 = -(phi - psi);
    float theta2 = -(phi + psi);

    float pos1_rad = theta1 + OFFSET_ANGLE;
    float pos2_rad = theta2 + OFFSET_ANGLE;

    const float JOINT_MAX = 1.4f;
    const float JOINT_MIN = -1.2f;
    pos1_rad = clamp_f(pos1_rad, JOINT_MIN, JOINT_MAX);
    pos2_rad = clamp_f(pos2_rad, JOINT_MIN, JOINT_MAX);

    if (leg_idx < 2) {
        motor_controlmode(&mi_motor_can1[leg_idx * 2],     0.0f, pos1_rad, 0.0f, g->kp, g->kd);
        motor_controlmode(&mi_motor_can1[leg_idx * 2 + 1], 0.0f, pos2_rad, 0.0f, g->kp, g->kd);
    } else {
        uint8_t base = (leg_idx - 2) * 2;
        motor_controlmode(&mi_motor_can2[base],     0.0f, -pos1_rad, 0.0f, g->kp, g->kd);
        motor_controlmode(&mi_motor_can2[base + 1], 0.0f, -pos2_rad, 0.0f, g->kp, g->kd);
    }
}

JumpController* jump_get_controller(void)
{
    return &g_jump;
}

void jump_trigger(void)
{
    if (g_jump.state != JUMP_IDLE) return;
    g_jump.state = JUMP_CROUCH;
    g_jump.timer = 0.0f;
}

uint8_t jump_is_active(void)
{
    return (g_jump.state != JUMP_IDLE);
}

void jump_set_gains(JumpState state, float kp, float kd)
{
    if (state >= 7) return;
    g_jump.gains[state].kp = kp;
    g_jump.gains[state].kd = kd;
}

void jump_abort(void)
{
    g_jump.state = JUMP_IDLE;
    g_jump.timer = 0.0f;
}

void jump_check_remote_trigger(remoter_t *remote)
{
    if (!remote->online) {
        btn_was_pressed = 0;
        return;
    }

    uint8_t btn_pressed = (remote->rc.ch[6] > 500) ? 1 : 0;

    if (btn_pressed && !btn_was_pressed) {
        jump_trigger();
    }

    btn_was_pressed = btn_pressed;
}

void jump_update(float dt)
{
    if (g_jump.state == JUMP_IDLE) return;

    g_jump.timer += dt;
    JumpGains *g = &g_jump.gains[g_jump.state];
    JumpParams *p = &g_jump.params;

    switch (g_jump.state) {

        case JUMP_CROUCH: {
            float progress = g_jump.timer / 0.4f;
            if (progress >= 1.0f) {
                g_jump.state = JUMP_LEAN_FORWARD;
                g_jump.timer = 0.0f;
                break;
            }
            float ease = smoothstep(progress);
            float h = 213.2f + (p->crouch_height - 213.2f) * ease;
            for (uint8_t i = 0; i < 4; i++) {
                leg_control_with_gains(i, 0.0f, h, g);
            }
            break;
        }

        case JUMP_LEAN_FORWARD: {
            float progress = g_jump.timer / 0.25f;
            if (progress >= 1.0f) {
                g_jump.state = JUMP_EXPLODE;
                g_jump.timer = 0.0f;
                break;
            }
            float ease = smoothstep(progress);
            float offset = p->lean_angle * 0.8f * ease;
            leg_control_with_gains(0, -offset, p->crouch_height, g);
            leg_control_with_gains(1,  offset, p->crouch_height, g);
            leg_control_with_gains(2,  offset, p->crouch_height, g);
            leg_control_with_gains(3, -offset, p->crouch_height, g);
            break;
        }

        case JUMP_EXPLODE: {
            float progress = g_jump.timer / 0.12f;
            if (progress >= 1.0f) {
                g_jump.state = JUMP_AERIAL;
                g_jump.timer = 0.0f;
                break;
            }
            float curve = progress * progress * progress;
            float h = p->crouch_height + (175.0f - p->crouch_height) * curve * p->jump_power;
            float push = p->push_distance * curve;
            leg_control_with_gains(0, -push * 0.8f, h, g);
            leg_control_with_gains(1,  push * 0.8f, h, g);
            leg_control_with_gains(2,  push * 0.8f, h, g);
            leg_control_with_gains(3, -push * 0.8f, h, g);
            break;
        }

        case JUMP_AERIAL: {
            float progress = g_jump.timer / 0.15f;
            if (progress >= 1.0f) {
                g_jump.state = JUMP_LAND;
                g_jump.timer = 0.0f;
                break;
            }
            float tuck = 15.0f * progress;
            leg_control_with_gains(0, -tuck, p->aerial_height, g);
            leg_control_with_gains(1,  tuck, p->aerial_height, g);
            leg_control_with_gains(2,  tuck, p->aerial_height, g);
            leg_control_with_gains(3, -tuck, p->aerial_height, g);
            break;
        }

        case JUMP_LAND: {
            float progress = g_jump.timer / 0.25f;
            if (progress >= 1.0f) {
                g_jump.state = JUMP_RECOVER;
                g_jump.timer = 0.0f;
                break;
            }
            float h = p->aerial_height + (p->crouch_height - p->aerial_height) * progress * p->land_buffer;
            float lean = p->lean_angle * 0.8f * (1.0f - progress);
            leg_control_with_gains(0, -lean, h, g);
            leg_control_with_gains(1,  lean, h, g);
            leg_control_with_gains(2,  lean, h, g);
            leg_control_with_gains(3, -lean, h, g);
            break;
        }

        case JUMP_RECOVER: {
            float progress = g_jump.timer / 0.35f;
            if (progress >= 1.0f) {
                g_jump.state = JUMP_IDLE;
                g_jump.timer = 0.0f;
                break;
            }
            float ease = smoothstep(progress);
            float h = p->crouch_height + (213.2f - p->crouch_height) * ease;
            for (uint8_t i = 0; i < 4; i++) {
                leg_control_with_gains(i, 0.0f, h, g);
            }
            break;
        }

        default:
            break;
    }
}
