#ifndef __DCM_H
#define __DCM_H

#include "stm32f10x.h"
#include "mpu6050.h"
//#include "math.h"

#define Kp_ROLLPITCH_L	1.0f
#define Kp_ROLLPITCH	1.0f
//#define Ki_ROLLPITCH	0.00001f

extern float Kp_ROLLPITCH_curr;

void DCM_normalize(void);
void DCM_drift_corr(void);
void DCM_update(void);
void DCM_cal_euler(void);

#endif /* __DCM_H */
