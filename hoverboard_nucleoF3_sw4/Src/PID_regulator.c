/*
 * PID_regulator.c
 *
 *  Created on: 15.05.2018
 *      Author: Norbi
 */

#include "PID_regulator.h"
#include "stm32f3xx_hal.h"

#define ERR_SUM_MAX		250

int16_t outPidLimit = 250;

double Kp = 10;					// (P)roportional Tuning Parameter
double Ki = 0.05;					// (I)ntegral Tuning Parameter
double Kd = 2;					// (D)erivative Tuning Parameter

//max angle will be -40 & +40
//float last_angle;				// Keeps track of error over time
//float iTerm;					// Used to accumulate error (integral)


//float lastOut = 16;

//float targetAngle = 0;		// Can be adjusted according to centre of gravity


int16_t PID_calculate(PID_OBJ* pid_obj, float set_angle, float angle)
{

	pid_obj->PID_Result = 0;
    // Calculate time since last time PID was called (determine by timer6 (5ms), should be the same in Kalman)
	pid_obj->this_Time = HAL_GetTick();
	pid_obj->timeChange = pid_obj->this_Time - pid_obj->last_Time;
 //   lastOut = timeChange;

	// Calculate Error
	pid_obj->error = set_angle - angle;

	// Calculate our PID terms
	pid_obj->pTerm = Kp * pid_obj->error;
	pid_obj->iTerm += Ki * pid_obj->error * pid_obj->timeChange;
	pid_obj->dTerm = Kd * (angle - pid_obj->last_angle) / pid_obj->timeChange;

	if (pid_obj->iTerm > ERR_SUM_MAX) {
		pid_obj->iTerm = ERR_SUM_MAX;
	} else if (pid_obj->iTerm < -ERR_SUM_MAX) {
		pid_obj->iTerm = -ERR_SUM_MAX;
	}

	pid_obj->last_angle = angle;
	pid_obj->last_Time = pid_obj->this_Time;

    // Set PWM Value
	pid_obj->PID_Value = pid_obj->pTerm + pid_obj->iTerm - pid_obj->dTerm;

    // Limits PID to max motor speed
    if (pid_obj->PID_Value > outPidLimit) pid_obj->PID_Value = outPidLimit;
    else if (pid_obj->PID_Value < -outPidLimit) pid_obj->PID_Value = -outPidLimit;

    // Return PID Output
    pid_obj->PID_Result = (int16_t) pid_obj->PID_Value;
    return pid_obj->PID_Result;

}
