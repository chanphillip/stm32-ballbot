#include "mpu6050.h"

#define PI	3.1416f

#define GYRO_SCALE		16.4f
#define ACCEL_SCALE		8192

#define MPU6050_ACCEL_XOUT_H		0x3B
#define MPU6050_ACCEL_XOUT_L		0x3C
#define MPU6050_ACCEL_YOUT_H		0x3D
#define MPU6050_ACCEL_YOUT_L		0x3E
#define MPU6050_ACCEL_ZOUT_H		0x3F
#define MPU6050_ACCEL_ZOUT_L		0x40
#define MPU6050_GYRO_XOUT_H			0x43
#define MPU6050_GYRO_XOUT_L			0x44
#define MPU6050_GYRO_YOUT_H			0x45
#define MPU6050_GYRO_YOUT_L			0x46
#define MPU6050_GYRO_ZOUT_H			0x47
#define MPU6050_GYRO_ZOUT_L			0x48

#define GYRO_TH				5

s16 accel_raw[3];
float accel_data[3];
//float accel_mag;
//float accel_angle[3];

s16 gyro_raw[3];
float gyro_data[3];
float gyro_data_prev[3];
//float gyro_angle[3];		// xz, yz, yaw

float dcm_matrix[3][3];

s32 r_pitch;
s32 r_roll;
s32 r_yaw;

u16 MPU6050_init_cnt = 100;			// 1000 ms

s16 gyro_shift[3] = {28, -13, 28};
s16 accel_range_pos[3] = {8192, 8192, 8192};
s16 accel_range_neg[3] = {8192, 8192, 8192};

u16 ticks_img2 = 0;

/*float db1[2] = {0, 0};
float db2[2] = {0, 0};
float db3[2] = {0, 0};*/

u16 abs(s16 x) {
	return x < 0 ? -x : x;
}

void MPU6050_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef	I2C_InitStructure;
	
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	
	/* Configure I2C_EE pins: SCL and SDA */
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	
	/* I2C configuration */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = MPU6050_I2C_SLAVE_ADDRESS;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = MPU6050_I2C_Speed;
	
	/* I2C Peripheral Enable */
	I2C_Cmd(I2C2, ENABLE);
	/* Apply I2C configuration after enabling it */
	I2C_Init(I2C2, &I2C_InitStructure);
	
	//----init mpu
	MPU6050_write(MPU6050_PWR_MGMT_1, 0x00);
	MPU6050_write(MPU6050_CONFIG, 0x03);
	MPU6050_write(MPU6050_GYRO_CONFIG, 0x18);
	MPU6050_write(MPU6050_ACCEL_CONFIG, 0x08);
	MPU6050_write(MPU6050_SMPRT_DIV, 0x00);
	//MPU6050_write(MPU6050_PWR_MGMT_1, 0x00);
	
	
	accel_raw[0] = 0;
	accel_raw[1] = 0;
	accel_raw[2] = 0;
	accel_data[0] = 0;
	accel_data[1] = 0;
	accel_data[2] = 0;
	/*accel_mag = 0;
	accel_angle[0] = 0;
	accel_angle[1] = 0;
	accel_angle[2] = 0;*/
	
	gyro_raw[0] = 0;
	gyro_raw[1] = 0;
	gyro_raw[2] = 0;
	gyro_data[0] = 0;
	gyro_data[1] = 0;
	gyro_data[2] = 0;
	gyro_data_prev[0] = 0;
	gyro_data_prev[1] = 0;
	gyro_data_prev[2] = 0;
	/*gyro_angle[0] = 0;
	gyro_angle[1] = 0;
	gyro_angle[2] = 0;*/
	
	dcm_matrix[0][0] = 1;
	dcm_matrix[0][1] = 0;
	dcm_matrix[0][2] = 0;
	dcm_matrix[1][0] = 0;
	dcm_matrix[1][1] = 1;
	dcm_matrix[1][2] = 0;
	dcm_matrix[2][0] = 0;
	dcm_matrix[2][1] = 0;
	dcm_matrix[2][2] = 1;
	
	r_pitch = 0;
	r_roll = 0;
	r_yaw = 0;
	
	
	MPU6050_timer_init();
	
}

void MPU6050_write(u8 addr, u8 data)
{
	while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));
	
	I2C_GenerateSTART(I2C2, ENABLE);
	while (I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS);
	
	I2C_Send7bitAddress(I2C2, MPU6050_I2C_SLAVE_ADDRESS, I2C_Direction_Transmitter);
	while (I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS);
	
	I2C_SendData(I2C2, addr);
	while (I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS);
	
	I2C_SendData(I2C2, data);
	while (I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS);
	
	I2C_GenerateSTOP(I2C2, ENABLE);
	
}

u8 MPU6050_read(u8 addr)
{
	u8 readData = 0;
	MPU6050_read_nbyte(&readData, addr, 1);
	return readData;
}

void MPU6050_read_nbyte(u8* data, u8 addr, u8 n)
{
	u8 i = 0;
	
	while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));
	
	I2C_GenerateSTART(I2C2, ENABLE);
	while (I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS);
	
	I2C_Send7bitAddress(I2C2, MPU6050_I2C_SLAVE_ADDRESS, I2C_Direction_Transmitter);
	while (I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS);
	
	I2C_SendData(I2C2, addr);
	while (I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS);
	
	I2C_GenerateSTART(I2C2, ENABLE);
	while (I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS);
	
	I2C_Send7bitAddress(I2C2, MPU6050_I2C_SLAVE_ADDRESS, I2C_Direction_Receiver);
	while (I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) != SUCCESS);
	
	for (i = 0; i < n; ++i) {
		if (i == n-1) {
			I2C_AcknowledgeConfig(I2C2, DISABLE);
			I2C_GenerateSTOP(I2C2, ENABLE);
		}
		
		while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
		*data = I2C_ReceiveData(I2C2);
		++data;
	}
	
	I2C_AcknowledgeConfig(I2C2, ENABLE);
}

void MPU6050_accel_update(void)
{
	u8 tmp[6];
	MPU6050_read_nbyte(tmp, MPU6050_ACCEL_XOUT_H, 6);
	accel_raw[0] = (tmp[0] << 8) | tmp[1];
	accel_raw[1] = (tmp[2] << 8) | tmp[3];
	accel_raw[2] = (tmp[4] << 8) | tmp[5];
}

void MPU6050_gyro_update(void)
{
	u8 tmp[6];
	MPU6050_read_nbyte(tmp, MPU6050_GYRO_XOUT_H, 6);
	gyro_raw[0] = (tmp[0] << 8) | tmp[1];
	gyro_raw[1] = (tmp[2] << 8) | tmp[3];
	gyro_raw[2] = (tmp[4] << 8) | tmp[5];
}

void MPU6050_accel_cal(void)
{
	accel_data[0] = (float)-(accel_raw[0]+(accel_range_pos[0]+accel_range_neg[0]-ACCEL_SCALE*2))/ACCEL_SCALE;
	accel_data[1] = (float)-(accel_raw[1]+(accel_range_pos[1]+accel_range_neg[1]-ACCEL_SCALE*2))/ACCEL_SCALE;
	accel_data[2] = (float)-(accel_raw[2]+(accel_range_pos[2]+accel_range_neg[2]-ACCEL_SCALE*2))/ACCEL_SCALE;
}

void MPU6050_gyro_cal(void)
{
	if (abs(gyro_raw[0]+gyro_shift[0]) <= GYRO_TH) {
		gyro_data[0] = 0;
	} else {
		gyro_data[0] = (gyro_raw[0]+gyro_shift[0])/GYRO_SCALE;
	}
	if (abs(gyro_raw[1]+gyro_shift[1]) <= GYRO_TH) {
		gyro_data[1] = 0;
	} else {
		gyro_data[1] = (gyro_raw[1]+gyro_shift[1])/GYRO_SCALE;
	}
	if (abs(gyro_raw[2]+gyro_shift[2]) <= GYRO_TH) {
		gyro_data[2] = 0;
	} else {
		gyro_data[2] = (gyro_raw[2]+gyro_shift[2])/GYRO_SCALE;
	}
	
}

void MPU6050_timer_init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);
	
	TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 1000;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM4, ENABLE);
}

void print_hex(float x) {
	float y = (float)x;
	u8* d2c = (u8*)&y;
	u8 i;
	for (i = 0; i < 4; ++i) {
		printf("%02X", *(d2c + 3 - i));
	}
}

void TIM4_IRQHandler(void)
{
	u8 tmp[14];
	
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

	MPU6050_read_nbyte(tmp, MPU6050_ACCEL_XOUT_H, 14);
	accel_raw[0] = (tmp[0] << 8) | tmp[1];
	accel_raw[1] = (tmp[2] << 8) | tmp[3];
	accel_raw[2] = (tmp[4] << 8) | tmp[5];
	gyro_raw[0] = (tmp[8] << 8) | tmp[9];
	gyro_raw[1] = (tmp[10] << 8) | tmp[11];
	gyro_raw[2] = (tmp[12] << 8) | tmp[13];
	
	MPU6050_accel_cal();
	MPU6050_gyro_cal();
	
	if (MPU6050_init_cnt > 0) {
		--MPU6050_init_cnt;
		if (!MPU6050_init_cnt) {
			Kp_ROLLPITCH_curr = Kp_ROLLPITCH;
		}
	}
	
	DCM_drift_corr();
	DCM_update();
	DCM_normalize();
	DCM_cal_euler();
	
	if (ticks_img2 != get_ticks()) {
		ticks_img2 = get_ticks();
		if (ticks_img2 % 50 == 0) {
			//x = (ticks/30) % 2;
			//printf("%.2f\t%.2f\t%.2f\t%.2f\r\n", r_pitch[0], r_roll[0], r_pitch[1], r_roll[1]);
			
			/*printf("%d\t%d\t%d\t%.2f\t%.2f\t%.2f", accel_raw[0], accel_raw[1], accel_raw[2], gyro_data[0], gyro_data[1], gyro_angle[2]);
			if (x == 0) {
				printf("\t\t");
			} else {
				printf("\t\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\r\n", r_pitch, r_roll, r_yaw, db1, db2, db3);
			}*/
			
			//if (x == 1) {
				//print_hex(accel_data[0]); printf(",");
			/*if (!MPU6050_init_cnt) {
				print_hex(accel_raw[0]); printf(",");		//accel_data[0]
				print_hex(accel_raw[1]); printf(",");
				print_hex(accel_raw[2]); printf(",");
				print_hex(gyro_raw[0]); printf(",");
				print_hex(gyro_raw[1]); printf(",");
				print_hex(gyro_raw[2]); printf(",");
				print_hex((float)r_pitch*PI/180); printf(",");
				print_hex((float)r_roll*PI/180); printf(",");
				print_hex((float)r_yaw*PI/180);
				printf("\n");
			}*/
				/*printf("%d,%.2f,%.2f\n", ttt, r_pitch, r_roll);
				ttt += 2;*/
			//}
			
		}
	}
	//printf("%d\t%d\t%d\t%d\t%d\t%d\r\n", accel_data[0], accel_data[1], accel_data[2], gyro_data[0], gyro_data[1], gyro_data[2]);
	
}
