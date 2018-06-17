/*
 * Uart.c
 *
 *  Created on: 17.06.2018
 *      Author: Norbi
 */

#include "Uart.h"
#include "test.h"

extern int32_t zmiennna_test;

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
	char *cr_array[10];
	int cr_counter = 0;

	uint8_t Data[40]; // Tablica przechowujaca wysylana wiadomosc.
	uint16_t size = 0; // Rozmiar wysylanej wiadomosci

//	split command to array
	cr_array[cr_counter] = strtok(input_comand, " ,.-");
	while (cr_array[cr_counter] != 0) {
		cr_array[++cr_counter] = strtok(0, " ,.-");
	}

	if (cr_counter == 1) {
		if (strcmp(input_comand, "stop") == 0) {
		size = sprintf(Data, "Stop\n\r");
	//	HAL_UART_Transmit_IT(&huart2, Data, size);

		}
	} else if (cr_counter == 2) {
		if (strcmp(input_comand, "speed") == 0) {
		size = sprintf(Data, "speed %s\n\r",cr_array[1]);
	//	HAL_UART_Transmit_IT(&huart2, Data, size);

		zmiennna_test = atoi(cr_array[1]);

		}
	} else {
		size = sprintf(Data, "Blep\n\r");
	//	HAL_UART_Transmit_IT(&huart2, Data, size);
	}

}

void UART_Command_Reading (void) {


	if (Transfer_cplt > 0) {
		Transfer_cplt = 0;
		comand_recognition(Rx_Buffer);
	}

}
