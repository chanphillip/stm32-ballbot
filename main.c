#include "main.h"

#define BACK_WHEEL	1
#define RIGHT_WHEEL	2
#define LEFT_WHEEL	0

#define SPEED_LIMIT_MAX		2500000
#define SPEED_LIMIT_MIN		0

#define KP		800
#define KI		0
#define KD		0

#define KP_M	0
#define KI_M	0
#define KD_M	0

#define X_ERROR_LIMIT	30000
#define Y_ERROR_LIMIT	30000

#define X_OFFSET	530
#define Y_OFFSET	815

#define IMU_OFFSET	0			// degrees scale by 10 anticlockwise

#define DT		0.005f

u16 ticks_img = 0;
u16 seconds_img = 0;

u8 starting = 0;
u8 command_mode = 0;
u8 command_type = 0;
u8 command_val_neg = 0;
s32 command_val = 0;

s32 wheel_b_vel = 0;
s32 wheel_r_vel = 0;
s32 wheel_l_vel = 0;

s32 x_velocity = 0;
s32 y_velocity = 0;
s32 x_cumulative_error = 0;
s32 y_cumulative_error = 0;
s32 x_velocity_pos = 0;
s32 y_velocity_pos = 0;

s32 x_curr_error = 0;
s32 y_curr_error = 0;
s32 x_prev_error = 0;
s32 y_prev_error = 0;

//------
float encoder_velocity_x = 0;
float encoder_velocity_y = 0;
float encoder_velocity_x_tmp = 0;
float encoder_velocity_y_tmp = 0;
float encoder_prev_velocity_x = 0;
float encoder_prev_velocity_y = 0;
float position_x = 0;
float position_y = 0;
s32 position_x_target = 0;
s32 position_y_target = 0;
s32 position_x_error = 0;
s32 position_y_error = 0;
s32 position_x_error_prev = 0;
s32 position_y_error_prev = 0;
s32 position_yaw_target = 0;
s32 position_x_cumulative_error = 0;
s32 position_y_cumulative_error = 0;

s32 encoder_b = 0;
s32 encoder_l = 0;
s32 encoder_r = 0;
s32 encoder_b_prev = 0;
s32 encoder_l_prev = 0;
s32 encoder_r_prev = 0;

s32 velocity_b = 0;
s32 velocity_l = 0;
s32 velocity_r = 0;

//------
s16 x_offset_increment = 552;
s16 y_offset_increment = -826;

s16 kp_increment = 63;
s16 ki_increment = 0;
s16 kd_increment = 1240;

s32 kp_m_increment = 3600;
s32 ki_m_increment = 0;
s32 kd_m_increment = 53800;

void omni_set_vel_analog(s32 vel_x, s32 vel_y, s32 vel_rotate)
{
	vel_x = vel_x/100;
	vel_y = vel_y/100;
	
	//for wheel 0(back wheel)
	wheel_b_vel = (vel_x * int_cos(IMU_OFFSET) - vel_y * int_sin(IMU_OFFSET)) / 100 + vel_rotate;		// 200000/10000
	//for wheel 0(left wheel)
	wheel_l_vel = (- vel_x * int_sin(IMU_OFFSET+300) - vel_y * int_cos(IMU_OFFSET+300)) / 100 + vel_rotate;
	//for wheel 1(right wheel)
	wheel_r_vel = (- vel_x * int_cos(IMU_OFFSET+600) + vel_y * int_sin(IMU_OFFSET+600)) / 100 + vel_rotate;

	/*wheel_b_vel = 100000;
	wheel_l_vel = 100000;
	wheel_r_vel = 100000;*/
	
	if (wheel_b_vel > SPEED_LIMIT_MAX) {
		wheel_b_vel = SPEED_LIMIT_MAX;
	} else if (wheel_b_vel < -SPEED_LIMIT_MAX) {
		wheel_b_vel = -SPEED_LIMIT_MAX;
	} else if (wheel_b_vel < SPEED_LIMIT_MIN && wheel_b_vel > -SPEED_LIMIT_MIN) {
		wheel_b_vel = 0;
	}
	
	if (wheel_r_vel > SPEED_LIMIT_MAX) {
		wheel_r_vel = SPEED_LIMIT_MAX;
	} else if (wheel_r_vel < -SPEED_LIMIT_MAX) {
		wheel_r_vel = -SPEED_LIMIT_MAX;
	} else if (wheel_r_vel < SPEED_LIMIT_MIN && wheel_r_vel > -SPEED_LIMIT_MIN) {
		wheel_r_vel = 0;
	}
	
	if (wheel_l_vel > SPEED_LIMIT_MAX) {
		wheel_l_vel = SPEED_LIMIT_MAX;
	} else if (wheel_l_vel < -SPEED_LIMIT_MAX) {
		wheel_l_vel = -SPEED_LIMIT_MAX;
	} else if (wheel_l_vel < SPEED_LIMIT_MIN && wheel_l_vel > -SPEED_LIMIT_MIN) {
		wheel_l_vel = 0;
	}
	
	lm629_velocity_start(BACK_WHEEL, wheel_b_vel);
	lm629_velocity_start(RIGHT_WHEEL, wheel_r_vel);
	lm629_velocity_start(LEFT_WHEEL, wheel_l_vel);
}	

void update_error()
{
	x_prev_error = x_curr_error;
	
	x_curr_error = (s32)(dcm_matrix[2][1]*10000 + X_OFFSET + x_offset_increment);//int_sin((r_roll - ROLL_OFFSET));
	x_cumulative_error += x_curr_error;
	if (x_cumulative_error > X_ERROR_LIMIT)
		x_cumulative_error = X_ERROR_LIMIT;
	if (x_cumulative_error < -X_ERROR_LIMIT)
		x_cumulative_error = -X_ERROR_LIMIT;

	y_prev_error = y_curr_error;
	
	y_curr_error = (s32)(dcm_matrix[2][0]*10000 + Y_OFFSET + y_offset_increment);//int_sin((r_pitch - PITCH_OFFSET));
	y_cumulative_error += y_curr_error;
	if (y_cumulative_error > Y_ERROR_LIMIT)
		y_cumulative_error = Y_ERROR_LIMIT;
	if (y_cumulative_error < -Y_ERROR_LIMIT)
		y_cumulative_error = -Y_ERROR_LIMIT;
}

void distribute_velocity(void)
{
	
	x_velocity += (KP+kp_increment) * x_curr_error + (KI+ki_increment) * x_cumulative_error + (KD+kd_increment) * (x_curr_error - x_prev_error);
	y_velocity += (KP+kp_increment) * y_curr_error + (KI+ki_increment) * y_cumulative_error + (KD+kd_increment) * (y_curr_error - y_prev_error);

}

void distribute_velocity_pos(void)
{
	position_x_error_prev = position_x_error;
	position_y_error_prev = position_y_error;
	
	position_x_error = position_x_target - (s32)position_x;
	position_y_error = position_y_target - (s32)position_y;
	
	if (Abs(position_x_error) >= 25) {
		/*if (position_x_error > 0) {
			position_x_target = (s32)position_x + 50;
			position_x_error = 50;
		} else {
			position_x_target = (s32)position_x - 50;
			position_x_error = -50;
		}*/
		position_x_target = (s32)position_x;
		position_x_error = 0;
		buzzer_control(1, 1);
	}
	
	if (Abs(position_y_error) >= 25) {
		/*if (position_y_error > 0) {
			position_y_target = (s32)position_y + 50;
			position_y_error = 50;
		} else {
			position_y_target = (s32)position_y - 50;
			position_y_error = -50;
		}*/
		position_y_target = (s32)position_y;
		position_y_error = 0;
		buzzer_control(1, 1);
	}
	
	//------
	position_x_cumulative_error += position_x_error;
	if (position_x_cumulative_error > X_ERROR_LIMIT)
		position_x_cumulative_error = X_ERROR_LIMIT;
	if (position_x_cumulative_error < -X_ERROR_LIMIT)
		position_x_cumulative_error = -X_ERROR_LIMIT;
	
	position_y_cumulative_error += position_y_error;
	if (position_y_cumulative_error > Y_ERROR_LIMIT)
		position_y_cumulative_error = Y_ERROR_LIMIT;
	if (position_y_cumulative_error < -Y_ERROR_LIMIT)
		position_y_cumulative_error = -Y_ERROR_LIMIT;
	
	//------
	x_velocity_pos = (KP_M+kp_m_increment) * position_x_error + (KI_M+ki_m_increment) * position_x_cumulative_error/1000 + (KD_M+kd_m_increment) * (position_x_error - position_x_error_prev);
	y_velocity_pos = (KP_M+kp_m_increment) * position_y_error + (KI_M+ki_m_increment) * position_y_cumulative_error/1000 + (KD_M+kd_m_increment) * (position_y_error - position_y_error_prev);
	
	x_velocity += x_velocity_pos;
	y_velocity += y_velocity_pos;

}

void position_calculation(void)
{
	float radian_angle = (r_yaw-position_yaw_target)*PI/180;
	float cos_a = cos(radian_angle);
	float sin_a = sin(radian_angle);
	
	encoder_b_prev = encoder_b;
	encoder_l_prev = encoder_l;
	encoder_r_prev = encoder_r;
	
	encoder_b = lm629_read_pos(BACK_WHEEL);
	encoder_l = lm629_read_pos(LEFT_WHEEL);
	encoder_r = lm629_read_pos(RIGHT_WHEEL);
	
	velocity_b = (float)(encoder_b - encoder_b_prev)*2.244;
	velocity_l = (float)(encoder_l - encoder_l_prev)*2.244;
	velocity_r = (float)(encoder_r - encoder_r_prev)*2.244;
	
	encoder_velocity_x_tmp = 2*cos(radian_angle)*velocity_b/3 - (cos(radian_angle)+sqrt(3)*sin(radian_angle))*velocity_r/3
						 - (cos(radian_angle)-sqrt(3)*sin(radian_angle))*velocity_l/3;
	encoder_velocity_y_tmp = 2*sin(radian_angle)*velocity_b/3 + (sqrt(3)*cos(radian_angle)-sin(radian_angle))*velocity_r/3
						 - (sqrt(3)*cos(radian_angle)+sin(radian_angle))*velocity_l/3;
	
	encoder_velocity_x = (encoder_velocity_x_tmp*int_cos(IMU_OFFSET) + encoder_velocity_y_tmp*int_sin(IMU_OFFSET))/10000;
	encoder_velocity_y = (encoder_velocity_x_tmp*int_sin(IMU_OFFSET) + encoder_velocity_y_tmp*int_cos(IMU_OFFSET))/10000;
	
	/*wheel_b_vel = (vel_x * int_cos(IMU_OFFSET) - vel_y * int_sin(IMU_OFFSET)) / 100 + vel_rotate;		// 200000/10000
	wheel_l_vel = (- vel_x * int_sin(IMU_OFFSET+300) - vel_y * int_cos(IMU_OFFSET+300)) / 100 + vel_rotate;
	wheel_r_vel = (- vel_x * int_cos(IMU_OFFSET+600) + vel_y * int_sin(IMU_OFFSET+600)) / 100 + vel_rotate;*/
	
	//encoder_velocity_x = velocity_b*int_cos(IMU_OFFSET) - velocity_l*int_sin(IMU_OFFSET+300) - velocity_r*int_cos(IMU_OFFSET+600);
	//encoder_velocity_y = -velocity_b*int_sin(IMU_OFFSET) - velocity_l*int_cos(IMU_OFFSET+300) + velocity_r*int_sin(IMU_OFFSET+600);

	encoder_velocity_x_tmp = (encoder_prev_velocity_x + encoder_velocity_x)/2*DT;
	encoder_velocity_y_tmp = (encoder_prev_velocity_y + encoder_velocity_y)/2*DT;

	position_x = position_x - encoder_velocity_x_tmp*cos_a-encoder_velocity_y_tmp*sin_a;
	position_y = position_y - encoder_velocity_y_tmp*cos_a-encoder_velocity_x_tmp*sin_a;
}

void toggle_start(void) {
	starting = !starting;
	if (starting) {
		
		x_curr_error = 0;
		x_prev_error = 0;
		x_cumulative_error = 0;
		
		y_curr_error = 0;
		y_prev_error = 0;
		y_cumulative_error = 0;
		
		x_velocity = 0;
		y_velocity = 0;
		
		position_x = 0;
		position_y = 0;
		
		position_x_cumulative_error = 0;
		position_y_cumulative_error = 0;
		
		position_yaw_target = r_yaw;

		buzzer_control(4, 3);
	} else {
		lm629_zero_drive(BACK_WHEEL);
		lm629_zero_drive(LEFT_WHEEL);
		lm629_zero_drive(RIGHT_WHEEL);
		buzzer_control(1, 20);
	}
}

void ballbot(void)
{
	tft_clear();
	tft_prints(0, 0, "Loading...");
	tft_update();
	
	_delay_ms(200);
	
	MPU6050_init();
	
	starting = 0;
	toggle_start();
	
	while (1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			if (ticks_img % 50 == 0) {
				tft_clear();
				tft_prints(0, 0, "P: %6d", r_pitch);
				tft_prints(0, 1, "R: %6d", r_roll);
				tft_prints(0, 2, "Y: %6d", r_yaw-position_yaw_target);
				
				/*tft_prints(0, 4, "B: %9d", wheel_b_vel);
				tft_prints(0, 5, "L: %9d", wheel_l_vel);
				tft_prints(0, 6, "R: %9d", wheel_r_vel);*/
				
				tft_prints(0, 3, "DX: %9d", x_curr_error);
				tft_prints(0, 4, "DY: %9d", y_curr_error);
				
				tft_prints(0, 5, "EX: %7d", position_x_error);
				tft_prints(0, 6, "EY: %7d", position_y_error);
				tft_prints(0, 7, "dx: %9d", x_velocity_pos);
				tft_prints(0, 8, "dy: %9d", y_velocity_pos);
				
				tft_prints(15, 9, "[%d]", get_seconds()%10);
				tft_update();
				
				button_update();
				if (button_down & START) {
					toggle_start();
				}
			}
			
			if (ticks_img % 2 == 0) {
				
				/*button_update();
				if (button_data & UP) {
					omni_set_vel_analog(0, 100000, 0);
				} else if (button_data & DOWN) {
					omni_set_vel_analog(0, -100000, 0);
				} else if (button_data & LEFT) {
					omni_set_vel_analog(-100000, 0, 0);
				} else if (button_data & RIGHT) {
					omni_set_vel_analog(100000, 0, 0);
				} else {
					omni_set_vel_analog(0, 0, 0);
				}*/
				
				x_velocity = 0;
				y_velocity = 0;
				
				position_calculation();
				
				/*if (moving) {
					position_y_target = position_y+50;
				}*/
				
				update_error();
				distribute_velocity();
				distribute_velocity_pos();
				
				if (starting) {
					omni_set_vel_analog(x_velocity, y_velocity, 0);
				} else {
					lm629_zero_drive(BACK_WHEEL);
					lm629_zero_drive(LEFT_WHEEL);
					lm629_zero_drive(RIGHT_WHEEL);
				}
				
			}
		}
	}
}

int main(void)
{
	u8 i = 0;
	
	//uart_init(COM1, 115200);
	//uart_printf_enable(COM1);
	
	uart_init(COM1, 115200);
	uart_interrupt(COM1);
	
	uart_init(COM3, 115200);
	uart_interrupt(COM3);
	
	uart_printf_enable(COM3);

	//gyro_init();
	tft_init(2, WHITE, BLACK, RED);
	ticks_init();
	button_init();
	//servo_init();
	//motor_init();
	buzzer_init();
	led_init();
	battery_adc_init();
	//xbc_init();
	lm629_init();
	//-- order of inits may affect the prog!
	
	seconds_img = get_seconds();
	ticks_img = get_ticks();

	for (i = 0; i < MAX_CHIP; i++) {
		if (lm629_chip_available & (1 << i)) {
			lm629_set_filter(i, 85, 0, 1024, 2048);
			lm629_acceleration(i, 2000);
			lm629_define_home(i);
			//lm629_stop_abruptly(i);
		}
	}
	
	menu_add(1, "Ballbot", ballbot);
	menu_add(2, "______", blank);
	MENU_ADD_LM629_TEST;
	MENU_ADD_GYRO_TEST;
	MENU_ADD_SERVO_TEST;
	MENU_ADD_PWM_TEST;
	menu_add(8, "GPIOE Test", gpioe_test);
	menu_add(9, "Program Info", get_program_info);
	
	menu_start("Ballbot v1       ", 1000);
	
	while (1) {
	  	menu(1);
	}

}

void command(u8 rx_data)
{
	if (command_mode) {
		if (!command_type) {
			if (rx_data < 97) rx_data += 32;
			if (rx_data == 'p' || rx_data == 'i' || rx_data == 'd' || rx_data == 'm' || rx_data == 'b' || rx_data == 'n') {
				command_type = rx_data;
				printf("%c ", rx_data);
				buzzer_control(1, 1);
			} else {
				printf("[END]\r\n");
				buzzer_control(2, 1);
				command_mode = 0;
			}
		} else {
			if (rx_data == 13) {		// enter
				
				if (command_val_neg) command_val = -command_val;
				
				switch (command_type) {
					case 'p': kp_increment = command_val; break;
					case 'i': ki_increment = command_val; break;
					case 'd': kd_increment = command_val; break;
					case 'm': kp_m_increment = command_val; break;
					case 'b': ki_m_increment = command_val; break;
					case 'n': kd_m_increment = command_val; break;
				}
				command_mode = 0;
				buzzer_control(3, 1);
				printf("\r\n");
				
			} else if (rx_data == 45 || (rx_data >= 48 && rx_data <= 57)) {
				if (rx_data == 45) {
					command_val_neg = 1;
					printf("-");
				} else {
					command_val = command_val*10 + (rx_data-48);
					printf("%c", rx_data);
				}
				buzzer_control(1, 1);
			} else {
				printf("[END]\r\n");
				buzzer_control(2, 1);
				command_mode = 0;
			}
		}
	} else {
		switch (rx_data) {
			case 'P':
				kp_increment += 1;
				buzzer_control(1, 1);
				break;
			case 'p':
				kp_increment -= 1;
				buzzer_control(1, 1);
				break;
			case 'I':
				ki_increment += 1;
				buzzer_control(1, 1);
				break;
			case 'i':
				ki_increment -= 1;
				buzzer_control(1, 1);
				break;
			case 'D':
				kd_increment += 2;
				buzzer_control(1, 1);
				break;
			case 'd':
				kd_increment -= 2;
				buzzer_control(1, 1);
				break;
			case 'X':
				x_offset_increment += 2;
				buzzer_control(1, 1);
				break;
			case 'x':
				x_offset_increment -= 2;
				buzzer_control(1, 1);
				break;
			case 'Y':
				y_offset_increment += 2;
				buzzer_control(1, 1);
				break;
			case 'y':
				y_offset_increment -= 2;
				buzzer_control(1, 1);
				break;
			case 'M':
				kp_m_increment += 100;
				buzzer_control(1, 1);
				break;
			case 'm':
				kp_m_increment -= 100;
				buzzer_control(1, 1);
				break;
			case 'N':
				kd_m_increment += 100;
				buzzer_control(1, 1);
				break;
			case 'n':
				kd_m_increment -= 100;
				buzzer_control(1, 1);
				break;
			case 'B':
				ki_m_increment += 5;
				buzzer_control(1, 1);
				break;
			case 'b':
				ki_m_increment -= 5;
				buzzer_control(1, 1);
				break;
			case 'r':
				position_x = 0;
				position_y = 0;
				buzzer_control(1, 3);
				break;
			case '#':
				command_mode = 1;
				command_type = 0;
				command_val_neg = 0;
				command_val = 0;
				buzzer_control(1, 2);
				break;
			case 's':
				toggle_start();
				break;
		}
		printf("x:%d\ty:%d\tp:%d\ti:%d\td:%d\tmp:%d\tmi:%d\tmd:%d\r\n", x_offset_increment, y_offset_increment, kp_increment, ki_increment, kd_increment, kp_m_increment, ki_m_increment, kd_m_increment);
		if (command_mode) {
			printf("CMD = ");
		}
	}
}

void USART1_IRQHandler(void)
{
	u8 rx_data = 0;

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
		uart_printf_enable(COM1);
		rx_data = USART_ReceiveData(USART1);
		command(rx_data);
	}
}

void USART3_IRQHandler(void)
{
	u8 rx_data = 0;

	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
		uart_printf_enable(COM3);
		rx_data = USART_ReceiveData(USART3);
		command(rx_data);
	}
}
