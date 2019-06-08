/*
 * gps.h
 *
 *  Created on: 21/04/2019
 *  Author: edy_g
 *  brief:	Configuration and control gps header
 */

#ifndef GPS_H_
#define GPS_H_

#include "fsl_uart.h"
#include "pin_mux.h"
#include "fsl_common.h"
#include "fsl_port.h"

/* ********************* Macros module ********************* */

#define GPS_DATA_OK			0u
#define GPS_DATA_NO_OK		1u
#define GPS_DATA_TIMEOUT	2u


/* UART instance and clock */
#define GPS_UART UART4
#define GPS_UART_CLKSRC UART4_CLK_SRC
#define GPS_UART_CLK_FREQ CLOCK_GetFreq(UART4_CLK_SRC)
#define GPS_UART_IRQn UART4_RX_TX_IRQn
#define GPS_UART_IRQHandler UART4_RX_TX_IRQHandler

#define GPS_BAUDRATE	9600u




/* Variables module */
extern volatile uint16_t txIndex; 			/* Index of the data to send out. */
extern volatile uint16_t rxIndex; 			/* Index of the memory to save new arrived data. */
extern volatile uint16_t rxMessageIndex;	//Index of current message arrived
extern volatile float lflCurrentDistance;

typedef struct
{
	float Latitude;
	float Longitude;
	float Distance;
	unsigned int DateInSeconds;		//Current date in seconds since 1980
	float Speed;
}stGPSData_t;



/* ********************* Public functions ********************* */
void vfnInitGPS_UART(void);									//Configures UART 4 for communications to GPS
unsigned char vfnGetDataGPS(stGPSData_t * stDataGPS);	//Function to get current GPS data


#endif /* GPS_H_ */
