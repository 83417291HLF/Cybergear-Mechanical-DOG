#include "gait_planner.h"
#include "leg0_config.h"
#include "math.h"

static float g_phase = 0.0f;
static float g_step_length = 30.0f; // mm
static float g_height = 40.0f;      // mm
static float g_freq = 0.5f;         // Hz

void GaitPlanner_Init(void)
{
    g_phase = 0.0f;
}

void GaitPlanner_ComputeStep(float dt, float *out_x, float *out_z)
{
    if (!out_x || !out_z) return;

    // simple vertical sinusoid + small fore-aft shift
    g_phase += 2.0f * M_PI * g_freq * dt;
    if (g_phase > 2.0f * M_PI) g_phase -= 2.0f * M_PI;

    // center on initial foot position defined in leg0 config
    float cx = leg0.init_foot_x;
    float cz = leg0.init_foot_z;

    float offset_z = g_height * sinf(g_phase);
    float offset_x = (g_step_length/2.0f) * sinf(g_phase);

    *out_x = cx + offset_x;
    *out_z = cz + offset_z;
}
