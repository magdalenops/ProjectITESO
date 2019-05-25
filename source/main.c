/*
*         Copyright (c), NXP Semiconductors Caen / France
*
*                     (C)NXP Semiconductors
*       All rights are reserved. Reproduction in whole or in part is
*      prohibited without the written consent of the copyright owner.
*  NXP reserves the right to make changes without notice at any time.
* NXP makes no warranty, expressed, implied or statutory, including but
* not limited to any implied warranty of merchantability or fitness for any
*particular purpose, or that the use will not infringe any third party patent,
* copyright or trademark. NXP must not be liable for any loss or damage
*                          arising from its use.
*/

#define SYSVIEW_EN 1

#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include <nfc_task.h>
#include <gps_Task.h>
#include <gps.h>

#if SYSVIEW_EN
#include "SEGGER_SYSVIEW.h"
#include "SEGGER_RTT.h"
#include "SEGGER_SYSVIEW_FreeRTOS.h"
#endif
//-----------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------
#define TASK_NFC_STACK_SIZE		1024
#define TASK_NFC_STACK_PRIO		(configMAX_PRIORITIES - 1)

//stGPSData_t NewDataSt;

int main(void) {
    /* Init board hardware. */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    vfnInitGPS_UART();

#if SYSVIEW_EN
    SEGGER_SYSVIEW_Conf();
    printf("RTT block address is: 0x%x\n", &_SEGGER_RTT);
#endif
	printf("\nRunning the NXP-NCI project.\n");

	/* Create NFC task */
    if (xTaskCreate((TaskFunction_t) task_nfc,
    				(const char*) "NFC_task",
					TASK_NFC_STACK_SIZE,
					NULL,
					TASK_NFC_STACK_PRIO,
					NULL) != pdPASS)
    {
    	printf("Failed to create NFC task");
    }

    /* Create GPS task */
     if (xTaskCreate((TaskFunction_t) gpsTask,
					(const char*) "GPS_task",
					configMINIMAL_STACK_SIZE + 30,
					NULL,
					TASK_NFC_STACK_PRIO - 1,
					NULL) != pdPASS)
	{
		printf("Failed to create GPS task");
	}

    vTaskStartScheduler();
    while(1) {}
	return 0;
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
