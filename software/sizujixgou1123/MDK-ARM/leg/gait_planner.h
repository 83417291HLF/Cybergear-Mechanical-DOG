#ifndef __GAIT_PLANNER_H
#define __GAIT_PLANNER_H

void GaitPlanner_Init(void);
// dt: seconds since last call. Outputs desired foot x,z in mm (robot frame)
void GaitPlanner_ComputeStep(float dt, float *out_x, float *out_z);

#endif
