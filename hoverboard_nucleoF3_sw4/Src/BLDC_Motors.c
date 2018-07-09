/*
 * BLDC_Motors.c
 *
 *  Created on: 25.04.2018
 *      Author: Norbi
 */

#include "BLDC_Motors.h"

#define LEFT_MOTOR_PORT		GPIOC
#define RIGHT_MOTOR_PORT	GPIOC

#define L_MOT_FORWARD	HAL_GPIO_WritePin(LEFT_MOTOR_PORT, LEFT_MOTOR_DIR_Pin, GPIO_PIN_SET)
#define L_MOT_BACKWARD	HAL_GPIO_WritePin(LEFT_MOTOR_PORT, LEFT_MOTOR_DIR_Pin, GPIO_PIN_RESET)

#define R_MOT_FORWARD	HAL_GPIO_WritePin(RIGHT_MOTOR_PORT, RIGHT_MOTOR_DIR_Pin, GPIO_PIN_RESET)
#define R_MOT_BACKWARD	HAL_GPIO_WritePin(RIGHT_MOTOR_PORT, RIGHT_MOTOR_DIR_Pin, GPIO_PIN_SET)


enum directions {
	BACKWARD = 0,
	FORWARD,
	LEFT,
	RIGHT
};

enum motors_conf {
	NONE_MOTOR = 0,
	LEFT_MOTOR,
	RIGHT_MOTOR,
	BOTH_MOTORS
};

#define MOTOR_MIN_PWM		350

//PWM Range 0-1000
static TIM_HandleTypeDef* BLDC_MotorsTimer;
static uint32_t leftChannel;
static uint32_t rightChannel;

static uint16_t leftPower = 0;
static uint16_t rightPower = 0;


void Initialize_LR_Motors(TIM_HandleTypeDef* timer, uint32_t leftCh, uint32_t rightCh) {
	BLDC_MotorsTimer = timer;
	leftChannel = leftCh;
	rightChannel = rightCh;

	HAL_TIM_PWM_Start(BLDC_MotorsTimer, leftChannel);
	HAL_TIM_PWM_Start(BLDC_MotorsTimer, rightChannel);
}

void setPwmDuty(TIM_HandleTypeDef* timer, uint32_t channel, uint16_t duty) {
	switch(channel){
		case TIM_CHANNEL_1:
			timer->Instance->CCR1 = duty;
			break;
		case TIM_CHANNEL_2:
			timer->Instance->CCR2 = duty;
			break;
	}
}

void Set_LR_Motors_Speed(uint16_t left, uint16_t right) {

	setPwmDuty(BLDC_MotorsTimer, leftChannel, left);
	setPwmDuty(BLDC_MotorsTimer, rightChannel, right);
	leftPower = left;
	rightPower = right;
}

void Set_Left_Motor_Speed(uint16_t left) {
	setPwmDuty(BLDC_MotorsTimer, leftChannel, left);
	leftPower = left;
}

void Set_Right_Motor_Speed(uint16_t right) {
	setPwmDuty(BLDC_MotorsTimer, rightChannel, right);
	rightPower = right;
}

void Stop_LR_Motors(void) {

	setPwmDuty(BLDC_MotorsTimer, leftChannel, 0);
	setPwmDuty(BLDC_MotorsTimer, rightChannel, 0);
}

void Set_Left_Motor_Dir(uint8_t dir)
{
	switch (dir) {
	   case FORWARD:
			HAL_GPIO_WritePin(LEFT_MOTOR_PORT, LEFT_MOTOR_DIR_Pin, GPIO_PIN_SET); //Forward
	     break;
	   case BACKWARD:
			HAL_GPIO_WritePin(LEFT_MOTOR_PORT, LEFT_MOTOR_DIR_Pin, GPIO_PIN_RESET); //Backward
	     break;

	   default:
	     break;
	 }
}

void Set_Right_Motor_Dir(uint8_t dir)
{
	switch (dir) {
	   case FORWARD:
			HAL_GPIO_WritePin(RIGHT_MOTOR_PORT, RIGHT_MOTOR_DIR_Pin, GPIO_PIN_RESET); //Forward
	     break;
	   case BACKWARD:
			HAL_GPIO_WritePin(RIGHT_MOTOR_PORT, RIGHT_MOTOR_DIR_Pin, GPIO_PIN_SET); //Backward
	     break;

	   default:
	     break;
	 }
}

void Set_LR_Motors_Dir(uint8_t dir){

	switch (dir) {
	case BACKWARD:
		Set_Left_Motor_Dir(BACKWARD);
		Set_Right_Motor_Dir(BACKWARD);
		break;
	case FORWARD:
		Set_Left_Motor_Dir(FORWARD);
		Set_Right_Motor_Dir(FORWARD);
		break;
	case LEFT:
		Set_Left_Motor_Dir(BACKWARD);
		Set_Right_Motor_Dir(FORWARD);
		break;
	case RIGHT:
		Set_Left_Motor_Dir(FORWARD);
		Set_Right_Motor_Dir(BACKWARD);
		break;
	}
}

void Control_Motor_by_PID(int8_t motor, int16_t pid_value) {
	int16_t pid_value_s = pid_value;
	switch(motor){
		case LEFT_MOTOR:
			if(pid_value_s > 0){
				Set_Left_Motor_Dir(FORWARD);
			} else if (pid_value_s < 0) {
				Set_Left_Motor_Dir(BACKWARD);
				pid_value_s = -pid_value_s;
			}
			setPwmDuty(BLDC_MotorsTimer, leftChannel, pid_value_s);
			leftPower = pid_value_s;
			break;

		case RIGHT_MOTOR:
			if(pid_value_s > 0){
				Set_Right_Motor_Dir(FORWARD);
			} else if (pid_value_s < 0) {
				Set_Right_Motor_Dir(BACKWARD);
				pid_value_s = -pid_value_s;
			}
			setPwmDuty(BLDC_MotorsTimer, rightChannel, pid_value_s);
			rightPower = pid_value_s;
			break;
		case BOTH_MOTORS:
			if(pid_value_s > 0){
				Set_Left_Motor_Dir(FORWARD);
				Set_Right_Motor_Dir(FORWARD);
			} else if (pid_value_s < 0) {
				Set_Left_Motor_Dir(BACKWARD);
				Set_Right_Motor_Dir(BACKWARD);
				pid_value_s = -pid_value_s;
			}
			if(pid_value_s <MOTOR_MIN_PWM)
				pid_value_s = MOTOR_MIN_PWM;
			setPwmDuty(BLDC_MotorsTimer, leftChannel, pid_value_s);
			setPwmDuty(BLDC_MotorsTimer, rightChannel, pid_value_s);
			leftPower = pid_value_s;
			rightPower = pid_value_s;
			break;

	}

}
