#ifndef __JUMP_H
#define __JUMP_H

#include <stdint.h>
#include "sbus.h"

typedef enum {
    JUMP_IDLE = 0,
    JUMP_CROUCH,
    JUMP_LEAN_FORWARD,
    JUMP_EXPLODE,
    JUMP_AERIAL,
    JUMP_LAND,
    JUMP_RECOVER
} JumpState;

typedef struct {
    float kp;
    float kd;
} JumpGains;

typedef struct {
    float crouch_height;
    float lean_angle;
    float jump_power;
    float push_distance;
    float aerial_height;
    float land_buffer;
} JumpParams;

typedef struct {
    JumpState state;
    float timer;
    JumpParams params;
    JumpGains gains[7];
} JumpController;

JumpController* jump_get_controller(void);
void jump_trigger(void);
void jump_update(float dt);
uint8_t jump_is_active(void);
void jump_set_gains(JumpState state, float kp, float kd);
void jump_abort(void);
void jump_check_remote_trigger(remoter_t *remote);

#endif