#ifndef __LEG_H
#define __LEG_H

#include "cybergear.h"
#include "math.h"
#include "sbus.h"
#include <stdlib.h>

#ifndef clamp
#define clamp(v, min, max) (((v) < (min)) ? (min) : ((v) > (max) ? (max) : (v)))
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ================== 机械参数 ==================
#define L1        100.0f    // 大腿长度
#define L2        200.0f    // 小腿长度
#define FOOT_EXT   45.0f    
#define L2_EFF    (L2 + FOOT_EXT)
#define OFFSET_ANGLE (90.0f * M_PI / 180.0f)

// ================== 步态结构 ==================
typedef struct {
    float S;            // 前后位移步幅 (CH1)
    float side_s;       // 侧移幅度 (CH0) 
    float yaw_s;        // 转向步幅 (CH2)
    float H;            // 抬腿高度
    float T_cycle;      // 步态周期
    float y_base;       // 基准腿长 (高度)
    float timer;        // 时间累加器
    uint8_t stepping_in_place; 
	
	
} GaitConfig;

extern GaitConfig myGait;


// ================== API ==================
void quadruped_init(void);
void update_quadruped_gait(float dt);
void leg_set_position_idx(uint8_t leg_idx, float x, float y);
void update_control_from_remote(remoter_t *remote);
// 在 cybergear.h 末尾添加
void check_and_recover_motors(void);
#endif