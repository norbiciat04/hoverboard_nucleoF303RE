/*
 * Uart.c
 *
 *  Created on: 17.06.2018
 *      Author: Norbi
 */

#include "Uart.h"
#include "BLDC_Motors.h"

void Uart_Receive (void) {

	uint8_t i;
	if (Rx_indx == 0) {
		for (i = 0; i < 100; i++)
			Rx_Buffer[i] = 0; //clear Rx_Buffer before receiving new data
	}
	if (Rx_data[0] != 13) //if received data different from ascii 13 (enter)
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
	cr_array[cr_counter] = strtok(input_comand, " ,.-");
	while (cr_array[cr_counter] != 0) {
		cr_array[++cr_counter] = strtok(0, " ,.-");
	}


	switch (cr_counter) {

	case 1:		//command without arguments
		if (strcmp(input_comand, "MOT_STOP") == 0) {
			Stop_LR_Motors();
		}
		break;

	case 2:		//one argument command
		if (strcmp(input_comand, "L_PWM") == 0) {
			Set_Left_Motor_Speed(atoi(cr_array[1]));
		}
		if (strcmp(input_comand, "R_PWM") == 0) {
			Set_Right_Motor_Speed(atoi(cr_array[1]));
		}
		if (strcmp(input_comand, "L_DIR") == 0) {
			Set_Left_Motor_Dir(atoi(cr_array[1]));
		}
		if (strcmp(input_comand, "R_DIR") == 0) {
			Set_Right_Motor_Dir(atoi(cr_array[1]));
		}
		if (strcmp(input_comand, "MOT_DIR") == 0) {
			Set_LR_Motors_Dir(atoi(cr_array[1]));
		}
		break;

	case 3:		//two arguments command
		if (strcmp(input_comand, "MOT_PWM") == 0) {
			Set_LR_Motors_Speed(atoi(cr_array[1]), atoi(cr_array[2]));
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
