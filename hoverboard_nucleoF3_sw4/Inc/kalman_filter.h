#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_



typedef struct KALMAN_OBJ {

	float x_post[2];

	float std_dev_v;
	float std_dev_w;

	float A[4], B[2], C[2];
	float V[4], W[1];
	float P_pri[4], P_post[4];
	float x_pri[2];
	float eps[1], S[1], K[2];
	float u[1], y[1];

	float Ax[2], Bu[2];
	float AP[4], AT[4], APAT[4];
	float Cx[1];
	float CP[2], CPCT[1];
	float PCT[2], S1[1];
	float Keps[2];
	float KS[2], KSKT[2];

}KALMAN_OBJ;

KALMAN_OBJ K_MPU6050_0, K_MPU6050_5, K_MPU6050_6, K_MPU6050_1;

void kalman_filter_init(KALMAN_OBJ* sensor, float acc_1, float acc_2);

float angle_before_kalman(float acc_1, float acc_2);
float kalman_filter_get_est(KALMAN_OBJ* sensor, float acc_1, float acc_2, float gyro);

#endif /* KALMAN_FILTER_H_ */
