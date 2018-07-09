/*
 * PID_regulator.c
 *
 *  Created on: 15.05.2018
 *      Author: Norbi
 */

#include "PID_regulator.h"
#include "stm32f3xx_hal.h"

#define ERR_SUM_MAX		1000

float Kp = 10;					// (P)roportional Tuning Parameter
float Ki = 0;					// (I)ntegral Tuning Parameter
float Kd = 0;					// (D)erivative Tuning Parameter

//max angle will be -40 & +40
float last_angle;				// Keeps track of error over time
float iTerm;					// Used to accumulate error (integral)

int16_t outMax = 1000;
int16_t outMin = -1000;

uint8_t lastOut = 16;

//float targetAngle = 0;		// Can be adjusted according to centre of gravity


int16_t PID_calculate(float set_angle, float angle)
{

	int16_t PID_Result = 0;
    // Calculate time since last time PID was called (determine by timer6 (5ms), should be the same in Kalman)
    unsigned long this_Time = HAL_GetTick();
    float timeChange = this_Time - last_Time;
    lastOut = timeChange;

	// Calculate Error
	float error = set_angle - angle;

	// Calculate our PID terms
	float pTerm = Kp * error;
	iTerm += Ki * error * timeChange;
	float dTerm = Kd * (angle - last_angle) / timeChange;

	if (iTerm > ERR_SUM_MAX) {
		iTerm = ERR_SUM_MAX;
	} else if (iTerm < -ERR_SUM_MAX) {
		iTerm = -ERR_SUM_MAX;
	}

	last_angle = angle;
	last_Time = this_Time;

    // Set PWM Value
    float PID_Value = pTerm + iTerm - dTerm;

    // Limits PID to max motor speed
    if (PID_Value > 1000) PID_Value = 1000;
    else if (PID_Value < -1000) PID_Value = -1000;

    // Return PID Output
    PID_Result = PID_Value;
    return PID_Result;

}
