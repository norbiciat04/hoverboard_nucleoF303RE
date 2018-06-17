/*
 * Uart.h
 *
 *  Created on: 17.06.2018
 *      Author: Norbi
 */

#ifndef UART_H_
#define UART_H_

#include "stm32f3xx_hal.h"


char Transfer_cplt, Rx_Buffer[100], Rx_indx, Rx_data[2];

void Uart_receive (void);
void comand_recognition(char *input_comand);
void UART_Command_Reading (void);

#endif /* UART_H_ */
