#include "dcm.h"

#define PI	3.1416f

float const toRad = PI/180;

float accel_vec[3] = { 0, 0, 0 }; //Store the acceleration in a vector
float gyro_vec[3] = { 0, 0, 0 };//Store the gyros turn rate in a vector
float omega_vec[3] = { 0, 0, 0 }; //Corrected gyro_vec data
float omega_p[3] = { 0, 0, 0 };//omega Proportional correction
//float omega_i[3] = { 0, 0, 0 };//omega Integrator
float omega[3] = { 0, 0, 0 };

float error_pr[3] = { 0, 0, 0 };		// error pitch roll

float Kp_ROLLPITCH_curr = Kp_ROLLPITCH_L;

float update_matrix[3][3] = {
	{ 0, 1, 2 },
	{ 3, 4, 5 },
	{ 6, 7, 8 } }; //Gyros here
float temporary_matrix[3][3] = {
	{ 0, 0, 0 },
	{ 0, 0, 0 },
	{ 0, 0, 0 } };

float g_dt = 0.001f; // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

float vec_dot(float v1[3], float v2[3])
{
	float r = 0;
	u8 i;
	for(i = 0; i < 3; ++i)
		r = r + v1[i]*v2[i];

	return r; 
}

void vec_cross(float vo[3], float v1[3], float v2[3])
{
	vo[0] = (v1[1]*v2[2]) - (v1[2]*v2[1]);
	vo[1] = (v1[2]*v2[0]) - (v1[0]*v2[2]);
	vo[2] = (v1[0]*v2[1]) - (v1[1]*v2[0]);
}

void vec_scale(float vo[3], float v1[3], float sc)
{
	u8 i;
	for (i = 0; i < 3; ++i)
		vo[i] = v1[i]*sc;
}

void vec_add(float vo[3], float v1[3], float v2[3])
{
	u8 i;
	for (i = 0; i < 3; ++i)
		vo[i] = v1[i]+v2[i];
}

void matrix_mul(float a[3][3], float b[3][3], float m[3][3])
{
	u8 i, j, k;
	float x[3];
	for (i = 0; i < 3; ++i) {
		for (j = 0; j < 3; ++j) {
			for (k = 0; k < 3; ++k) {
				x[k] = a[i][k]*b[k][j];
			}
			m[i][j] = x[0]+x[1]+x[2];
		}
	}
}


/**************************************************/
void DCM_normalize(void)
{
	float error = 0;
	float temporary[3][3];
	float renorm = 0;

	error = -vec_dot(&dcm_matrix[0][0], &dcm_matrix[1][0]) / 2; //eq.19

	vec_scale(&temporary[0][0], &dcm_matrix[1][0], error); //eq.19
	vec_scale(&temporary[1][0], &dcm_matrix[0][0], error); //eq.19

	vec_add(&temporary[0][0], &temporary[0][0], &dcm_matrix[0][0]);//eq.19
	vec_add(&temporary[1][0], &temporary[1][0], &dcm_matrix[1][0]);//eq.19

	vec_cross(&temporary[2][0], &temporary[0][0], &temporary[1][0]); // c= a x b //eq.20

	renorm = (3 - vec_dot(&temporary[0][0], &temporary[0][0])) / 2; //eq.21
	vec_scale(&dcm_matrix[0][0], &temporary[0][0], renorm);

	renorm = (3 - vec_dot(&temporary[1][0], &temporary[1][0])) / 2; //eq.21
	vec_scale(&dcm_matrix[1][0], &temporary[1][0], renorm);

	renorm = (3 - vec_dot(&temporary[2][0], &temporary[2][0])) / 2; //eq.21
	vec_scale(&dcm_matrix[2][0], &temporary[2][0], renorm);
}

/**************************************************/
void DCM_drift_corr(void)
{
	//Compensation the Roll, Pitch and Yaw drift.
	//float omega_p_new[3];
	//float omega_i_new[3];
	float accel_mag = 0;
	float accel_weight = 0;

	accel_vec[0] = accel_data[1];
	accel_vec[1] = accel_data[0];
	accel_vec[2] = -accel_data[2];
	
	//*****Roll and Pitch***************
	accel_mag = sqrt(accel_vec[0]*accel_vec[0] + accel_vec[1]*accel_vec[1] + accel_vec[2]*accel_vec[2]);
	
	// Dynamic weighting of accelerometer info (reliability filter)
	// Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
	accel_weight = 1 - 4*abs(1 - accel_mag);
	if (accel_weight < 0) {
		accel_weight = 0;
	} else if (accel_weight > 1) {
		accel_weight = 1;
	}

	vec_cross(&error_pr[0], &accel_vec[0], &dcm_matrix[2][0]); //adjust the ground of reference
	vec_scale(&omega_p[0], &error_pr[0], Kp_ROLLPITCH_curr * accel_weight);

	//vec_scale(&omega_i_new[0], &error_pr[0], Ki_ROLLPITCH * accel_weight);
	//vec_add(omega_i, omega_i, omega_i_new);
	
}
/**************************************************/

void DCM_update(void)
{
	u8 i, j;
	
	gyro_vec[0] = gyro_data[1]*toRad; //gyro x roll
	gyro_vec[1] = gyro_data[0]*toRad; //gyro y pitch
	gyro_vec[2] = -gyro_data[2]*toRad; //gyro Z yaw
	
	//vec_add(&omega[0], &gyro_vec[0], &omega_i[0]); //adding proportional term
	//vec_add(&omega_vec[0], &omega[0], &omega_p[0]); //adding Integrator term
	vec_add(&omega_vec[0], &gyro_vec[0], &omega_p[0]);

	update_matrix[0][0]= 0;
	update_matrix[0][1]= -g_dt*omega_vec[2];//-z
	update_matrix[0][2]= g_dt*omega_vec[1];//y
	update_matrix[1][0]= g_dt*omega_vec[2];//z
	update_matrix[1][1]= 0;
	update_matrix[1][2]= -g_dt*omega_vec[0];//-x
	update_matrix[2][0]= -g_dt*omega_vec[1];//-y
	update_matrix[2][1]= g_dt*omega_vec[0];//x
	update_matrix[2][2]= 0;

	matrix_mul(dcm_matrix, update_matrix, temporary_matrix); //a*b=c

	for (i = 0; i < 3; ++i) {	 	//Matrix Addition (update)
		for (j = 0; j < 3; ++j) {
			dcm_matrix[i][j] += temporary_matrix[i][j];
		}
	}
}

void DCM_cal_euler(void)
{
	//float tmp;
	//tmp = asin(dcm_matrix[2][0]);
	//r_pitch = (s32)(tmp*1800/PI);
	r_pitch = -int_arc_sin((s32)(dcm_matrix[2][0]*10000));
	//tmp = atan2(dcm_matrix[2][1], dcm_matrix[2][2]);
	//r_roll = (s32)(tmp*1800/PI);
	r_roll = int_arc_tan2((s32)(dcm_matrix[2][1]*10000), (s32)(dcm_matrix[2][2]*10000));
	r_yaw = -int_arc_tan2((s32)(dcm_matrix[1][0]*10000), (s32)(dcm_matrix[0][0]*10000));
}
