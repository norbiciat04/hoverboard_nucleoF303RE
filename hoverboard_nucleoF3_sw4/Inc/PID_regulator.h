/*
 * PID_regulator.h
 *
 *  Created on: 15.05.2018
 *      Author: Norbi
 */

#ifndef PID_REGULATOR_H_
#define PID_REGULATOR_H_

#include <stdio.h>

unsigned long last_Time;             // Time since PID was called last (should be ~10ms)




typedef struct PID_OBJ {

	int16_t PID_Result;
	unsigned long this_Time;
	unsigned long last_Time;
    float timeChange;

    float last_angle;				// Keeps track of error over time

    float error;
    float pTerm;
    float iTerm;
    float dTerm ;
    float PID_Value;

}PID_OBJ;

PID_OBJ PID_MOT_0, PID_MOT_1;


int16_t PID_calculate(PID_OBJ* pid_obj, float set_angle, float angle);


#endif /* PID_REGULATOR_H_ */
