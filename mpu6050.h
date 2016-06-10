#ifndef __MPU6050_H
#define __MPU6050_H

//#include "math.h"
#include "approx_math.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_tim.h"
#include "dcm.h"
#include "ticks.h"
#include "uart.h"

#define MPU6050_I2C_SLAVE_ADDRESS	0xD0
#define MPU6050_I2C_Speed			400000		// 100k-400k

#define MPU6050_CONFIG				0x1A		// Configuration
#define MPU6050_GYRO_CONFIG			0x1B		// Gyroscope Configuration		  //FS_SEL =  3
#define MPU6050_ACCEL_CONFIG		0x1C		// Accelerometer Configuration	  //AFS_SEL =  1
#define MPU6050_PWR_MGMT_1			0x6B		// Power Management 1   
#define MPU6050_SMPRT_DIV			0x19		// Sample Rate Divider

extern u16 MPU6050_init_cnt;

extern s16 accel_raw[3];
extern float accel_data[3];
//extern float accel_mag;
//extern float accel_ang[3];

extern s16 gyro_raw[3];
extern float gyro_data[3];
//extern float gyro_angle[3];

extern float dcm_matrix[3][3];

extern s32 r_pitch;
extern s32 r_roll;
extern s32 r_yaw;

void MPU6050_init(void);
void MPU6050_write(u8 addr, u8 data);
u8 MPU6050_read(u8 addr);
void MPU6050_read_nbyte(u8* data, u8 addr, u8 n);

void MPU6050_accel_cal(void);
void MPU6050_gyro_cal(void);

void MPU6050_accel_update(void);
void MPU6050_gyro_update(void);

void MPU6050_timer_init(void);

#endif /* __MAIN_H */
