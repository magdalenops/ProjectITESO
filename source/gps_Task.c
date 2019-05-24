/*
 * gps_Task.c
 *
 *  Created on: May 14, 2019
 *      Author: Mauricio
 */

#include <stdio.h>
#include <stdint.h>
#include <gps.h>



void gpsTask(void)
{
	stGPSData_t NewDataSt;
	uint8_t result=3;
	uint32_t timer = 0;
	for(uint8_t Cycle = 0; Cycle<100; Cycle++)
	{
		result = vfnGetDataGPS(&NewDataSt);
		printf("GPSDATA is: 0x%x\n", result);
		printf("GPSDATA Latitude is: 0x%f\n", NewDataSt.Latitude);
		printf("GPSDATA Longitude is: 0x%f\n", NewDataSt.Longitude);
		while (timer<0x02ffffff){timer++;}
		timer = 0;
	}
}

