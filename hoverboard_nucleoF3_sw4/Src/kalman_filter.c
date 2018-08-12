#include "kalman_filter.h"

#include "matrix.h"
#include <math.h>
//#include <stdint.h>

#define MPU6050_GYRO_SENS 250
#define MPU6050_ACCE_SENS 8


float dev_v = 5;
float dev_w = 1;

static float dt = 0.005;			//depend on timer interrupt time


void kalman_filter_init(KALMAN_OBJ* sensor, float acc_1, float acc_2)
{
	/*
	float acc_1_t = acc_1;
	float acc_2_t = acc_2;
*/
	/* Inicjalizacja zmiennych */
	sensor->A[0] = 1;
	sensor->A[1] = -dt;
	sensor->A[2] = 0;
	sensor->A[3] = 1;

	sensor->B[0] = dt;
	sensor->B[1] = 0;

	sensor->C[0] = 1;
	sensor->C[1] = 0;

	sensor->std_dev_v = dev_v;
	sensor->std_dev_w = dev_w;

	sensor->V[0] = sensor->std_dev_v*sensor->std_dev_v*dt;
	sensor->V[1] = 0;
	sensor->V[2] = 0;
	sensor->V[3] = sensor->std_dev_v*sensor->std_dev_v*dt;
	sensor->W[0] = sensor->std_dev_w*sensor->std_dev_w;

	/* Wartosci poczatkowe filtru */
	sensor->P_post[0] = 1;
	sensor->P_post[1] = 0;
	sensor->P_post[2] = 0;
	sensor->P_post[3] = 1;
/*
	acc_1_t = acc_1_t*MPU6050_ACCE_SENS/65535;
	acc_2_t = acc_2_t*MPU6050_ACCE_SENS/65535;
	*/
	sensor->x_post[0] = atan((acc_1*MPU6050_ACCE_SENS/65535)/(acc_2*MPU6050_ACCE_SENS/65535))*180/M_PI;
	sensor->x_post[1] = 0;
}

float angle_before_kalman(float acc_1, float acc_2)
{
	/*
    float angle;
    float acc_1_t = acc_1;
	float acc_2_t = acc_2;

    angle = atan(acc_1/acc_2)*180/M_PI;
    return angle;
    */
    return atan(acc_1/acc_2)*180/M_PI;
}

float kalman_filter_get_est(KALMAN_OBJ* sensor, float acc_1, float acc_2, float gyro)
{
	//Using: Two axis and angle perpendicular to them
	//Example acc_x, acc_y, gyro_z
	//Example acc_x, acc_z, gyro_y
/*
	float acc_1_t = acc_1;
	float acc_2_t = acc_2;
	float gyro_t = gyro;
*/
	/* x(t+1|t) = Ax(t|t) + Bu(t) */
	sensor->u[0] = gyro*MPU6050_GYRO_SENS/32768;
	matrix_2x2_mul_2x1(sensor->A, sensor->x_post, sensor->Ax);
	matrix_2x1_mul_1x1(sensor->B, sensor->u, sensor->Bu);
	matrix_2x1_add_2x1(sensor->Ax, sensor->Bu, sensor->x_pri);

	/* P(t+1|t) = AP(t|t)A^T + V */
	matrix_2x2_mul_2x2(sensor->A, sensor->P_post, sensor->AP);
	matrix_2x2_trans(sensor->A, sensor->AT);
	matrix_2x2_mul_2x2(sensor->AP, sensor->AT, sensor->APAT);
	matrix_2x2_add_2x2(sensor->APAT, sensor->V, sensor->P_pri);

	/* eps(t) = y(t) - Cx(t|t-1) */
/*
	acc_1_t = acc_1_t*MPU6050_ACCE_SENS/65535;
	acc_2_t = acc_2_t*MPU6050_ACCE_SENS/65535;
*/
	sensor->y[0] = atan((acc_1*MPU6050_ACCE_SENS/65535)/(acc_2*MPU6050_ACCE_SENS/65535))*180/M_PI;
	matrix_1x2_mul_2x1(sensor->C, sensor->x_pri, sensor->Cx);
	sensor->eps[0] = sensor->y[0] - sensor->Cx[0];

	/* S(t) = CP(t|t-1)C^T + W */
	matrix_1x2_mul_2x2(sensor->C, sensor->P_pri, sensor->CP);
	matrix_1x2_mul_2x1(sensor->C, sensor->C, sensor->CPCT);
	sensor->S[0] = sensor->CPCT[0] + sensor->W[0];

	/* K(t) = P(t|t-1)C^TS(t)^-1 */
	matrix_2x2_mul_2x1(sensor->P_pri, sensor->C, sensor->PCT);
	sensor->S1[0] = 1/(sensor->S[0]);
	matrix_2x1_mul_1x1(sensor->PCT, sensor->S1, sensor->K);

	/* x(t|t) = x(t|t-1) + K(t)eps(t) */
	matrix_2x1_mul_1x1(sensor->K, sensor->eps, sensor->Keps);
	matrix_2x1_add_2x1(sensor->x_pri, sensor->Keps, sensor->x_post);

	/* P(t|t) = P(t|t-1) - K(t)S(t)K(t)^T */
	matrix_2x1_mul_1x1(sensor->K, sensor->S, sensor->KS);
	matrix_2x1_mul_1x2(sensor->KS, sensor->K, sensor->KSKT);
	matrix_2x2_sub_2x2(sensor->P_pri, sensor->KSKT, sensor->P_post);


	return sensor->x_post[0];
}
