/*
 * Uart.c
 *
 *  Created on: 17.06.2018
 *      Author: Norbi
 */
#include <stdlib.h>
#include "Uart.h"
#include "BLDC_Motors.h"
#include "PID_regulator.h"


extern UART_HandleTypeDef huart2;

uint8_t str[50];
uint16_t size;

extern double Kp;
extern double Ki;
extern double Kd;

extern uint8_t allow;

void Uart_Receive (void) {

	uint8_t i;
	if (Rx_indx == 0) {
		for (i = 0; i < 100; i++)
			Rx_Buffer[i] = 0; //clear Rx_Buffer before receiving new data
	}
	if (Rx_data[0] != 10) //if received data different from ascii 10 (enter)
			{
		Rx_Buffer[Rx_indx++] = Rx_data[0]; //add data to Rx_Buffer
	} else      //if received data = 13
	{
		Rx_indx = 0;
		Transfer_cplt = 1; //transfer complete, data is ready to read
	}

}

void comand_recognition(char *input_comand) {
	char *cr_array[10];		//maximum command arguments
	uint8_t cr_counter = 0;		//counter of command arguments

	//	split command to array
	cr_array[cr_counter] = strtok(input_comand, " ");	// separating signs
	while (cr_array[cr_counter] != 0) {
		cr_array[++cr_counter] = strtok(0, " ");
	}


	switch (cr_counter) {

	case 1:		//command without arguments
		if (strcmp(cr_array[0], "MOT_STOP") == 0) {
			Stop_LR_Motors();
			size = sprintf(str, "Stop motors\r\n");
			HAL_UART_Transmit(&huart2, str, size, 1000);
		}
		if (strcmp(cr_array[0], "A") == 0) {
			allow=1;
			size = sprintf(str, "Allow=%d\r\n", allow);
			HAL_UART_Transmit(&huart2, str, size, 1000);
		}
		if (strcmp(cr_array[0], "NA") == 0) {
			allow=0;
			size = sprintf(str, "Allow=%d\r\n", allow);
			HAL_UART_Transmit(&huart2, str, size, 1000);
		}
		break;

	case 2:		//one argument command
		if (strcmp(cr_array[0], "L_PWM") == 0) {
			Set_Left_Motor_Speed(atoi(cr_array[1]));
			size = sprintf(str, "Left motor PWM=%d\r\n", atoi(cr_array[1]));
			HAL_UART_Transmit(&huart2, str, size, 1000);
		}
		if (strcmp(cr_array[0], "R_PWM") == 0) {
			Set_Right_Motor_Speed(atoi(cr_array[1]));
			size = sprintf(str, "Right motor PWM=%d\r\n", atoi(cr_array[1]));
			HAL_UART_Transmit(&huart2, str, size, 1000);
		}
		if (strcmp(cr_array[0], "L_DIR") == 0) {
			Set_Left_Motor_Dir(atoi(cr_array[1]));
			size = sprintf(str, "Left motor dir=%d\r\n", atoi(cr_array[1]));
			HAL_UART_Transmit(&huart2, str, size, 1000);
		}
		if (strcmp(cr_array[0], "R_DIR") == 0) {
			Set_Right_Motor_Dir(atoi(cr_array[1]));
			size = sprintf(str, "Right motor dir=%d\r\n", atoi(cr_array[1]));
			HAL_UART_Transmit(&huart2, str, size, 1000);
		}
		if (strcmp(cr_array[0], "MOT_DIR") == 0) {
			Set_LR_Motors_Dir(atoi(cr_array[1]));
			size = sprintf(str, "Motors dir=%d\r\n", atoi(cr_array[1]));
			HAL_UART_Transmit(&huart2, str, size, 1000);
		}
		if (strcmp(cr_array[0], "P") == 0) {
			Kp=atof(cr_array[1]);
			size = sprintf(str, "Kp=%s\r\n", cr_array[1]);
			HAL_UART_Transmit(&huart2, str, size, 1000);
		}
		if (strcmp(cr_array[0], "I") == 0) {
			Ki=atof(cr_array[1]);
			size = sprintf(str, "Ki=%s\r\n", cr_array[1]);
			HAL_UART_Transmit(&huart2, str, size, 1000);
		}
		if (strcmp(cr_array[0], "D") == 0) {
			Kd=atof(cr_array[1]);
			size = sprintf(str, "Kd=%s\r\n", cr_array[1]);
			HAL_UART_Transmit(&huart2, str, size, 1000);
		}
		break;

	case 3:		//two arguments command
		if (strcmp(cr_array[0], "MOT_PWM") == 0) {
			Set_LR_Motors_Speed(atoi(cr_array[1]), atoi(cr_array[2]));
			size = sprintf(str, "Motor PWM: left=%d right=%d\r\n", atoi(cr_array[1]), atoi(cr_array[2]));
			HAL_UART_Transmit(&huart2, str, size, 1000);
		}
		break;

	default:	//garbage

		break;
	}

}


void UART_Command_Reading (void) {


	if (Transfer_cplt > 0) {
		Transfer_cplt = 0;
		comand_recognition(Rx_Buffer);
	}

}
