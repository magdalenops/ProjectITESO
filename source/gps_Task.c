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



void gpsTask(void)
{
	stGPSData_t NewDataSt;
	uint8_t result=3;
	uint32_t timer = 0;
	while (1){
		result = vfnGetDataGPS(&NewDataSt);
		printf("GPSDATA is: 0x%x\n", result);
		//printf("GPSDATA Latitude is: 0x%f\n", NewDataSt.Latitude);
		//printf("GPSDATA Longitude is: 0x%f\n", NewDataSt.Longitude);
		while (timer<0x0002ffff){timer++;taskYIELD();}

		timer = 0;

	};
}

