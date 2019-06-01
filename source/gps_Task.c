/*
 * gps_Task.c
 *
 *  Created on: May 14, 2019
 *      Author: Mauricio
 */

#include <stdio.h>
#include <stdint.h>
#include <gps.h>
#include <FreeRTOS.h>
#include <task.h>
#include <matrix.h>
#include <fsl_debug_console.h>

stGPSData_t NewDataSt;

void gpsTask(void)
{
	uint8_t result=3;
	uint32_t timer = 0;
	uint8_t clockFlag = 0x0;
	while (1){
		result = vfnGetDataGPS(&NewDataSt);
		PRINTF("GPSDATA is: 0x%x\n", result);
		if ((result == 0x1 || result == 0x2 ) &! clockFlag)
		{
			matrix_send(MATRIX_PIC_WAIT_CLOCK);
			clockFlag = 0x1;
		} else if (result == 0x1) {clockFlag = 0x1;}
		//printf("GPSDATA Latitude is: 0x%f\n", NewDataSt.Latitude);
		//printf("GPSDATA Longitude is: 0x%f\n", NewDataSt.Longitude);
		while (timer<0x0002ffff){timer++;taskYIELD();}

		timer = 0;

	};
}

