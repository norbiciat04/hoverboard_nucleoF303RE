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

int16_t PID_calculate(float set_angle, float angle);


#endif /* PID_REGULATOR_H_ */
