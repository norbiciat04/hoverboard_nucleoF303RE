/*
 * BLDC_Motors.h
 *
 *  Created on: 25.04.2018
 *      Author: Norbi
 */

#ifndef BLDC_MOTORS_H_
#define BLDC_MOTORS_H_

#include "stm32f3xx_hal.h"
#include "stdbool.h"

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

void setPwmDuty(TIM_HandleTypeDef* timer, uint32_t channel, uint16_t duty);

void Initialize_LR_Motors(TIM_HandleTypeDef* timer, uint32_t leftCh, uint32_t rightCh);

void Stop_LR_Motors(void);

void Set_LR_Motors_Speed(uint16_t left, uint16_t right);
void Set_LR_Motors_Dir(uint8_t dir);

void Set_Left_Motor_Speed(uint16_t left);
void Set_Left_Motor_Dir(uint8_t dir);

void Set_Right_Motor_Speed(uint16_t right);
void Set_Right_Motor_Dir(uint8_t dir);

void Control_Motor_by_PID(int8_t motor, int16_t pid_value);

/*
int32_t getLedRedPower(void);
int32_t getLedGreenPower(void);
TODO - get rpm of motors
*/
#endif /* BLDC_MOTORS_H_ */
