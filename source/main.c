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

#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include <nfc_task.h>
#include <gps_Task.h>
#include <gps.h>
#include "TAG_ID_Control.h"

#include "matrix.h"


#include "fsl_debug_console.h"
 /*         VARIABLES         ----------------------------------------------------------------- FINAL PROYECT*/
TaskHandle_t NFC_handle;
/*                  ----------------------------------------------------------------- FINAL PROYECT*/
//-----------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------
#define TASK_NFC_STACK_SIZE		1024
#define TASK_NFC_STACK_PRIO		(configMAX_PRIORITIES - 1)

static void matrix_task(void *pvParameters);
//stGPSData_t NewDataSt;

int main(void) {
    /* Init board hardware. */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    vfnInitGPS_UART();


    /*         Function initialization for GPIOS   ----------------------------------------------------------------- FINAL PROYECT*/
    /*  Function initialization for GPIOS */
    GPIOinit();
    flash_init();
    /*                                         ----------------------------------------------------------------- FINAL PROYECT*/

	printf("\nRunning the NXP-NCI project.\n");

	/* Create NFC task */
    if (xTaskCreate((TaskFunction_t) task_nfc,
    				(const char*) "NFC_task",
					TASK_NFC_STACK_SIZE,
					NULL,
					TASK_NFC_STACK_PRIO,
					NULL) != pdPASS)
    {
    	PRINTF("Failed to create NFC task");
    }

    /* Matrix task */
    if (xTaskCreate((TaskFunction_t)matrix_task, "MATRIX_TASK", 60u, NULL, configMAX_PRIORITIES, NULL) != pdPASS) {
    	PRINTF("MATRIX Task creation failed!.\n");
	}

    /* Create GPS task */
     if (xTaskCreate((TaskFunction_t) gpsTask,
					(const char*) "GPS_task",
					configMINIMAL_STACK_SIZE + 50,
					NULL,
					TASK_NFC_STACK_PRIO - 1,
					NULL) != pdPASS)
	{
    	 PRINTF("Failed to create GPS task");
	}

    vTaskStartScheduler();
    while(1) {}
	return 0;
}

static void matrix_task(void *pvParameters) {
	/* Initialize SPI */
	matrix_spi_init();
	gpio_pin_config_t matrix_power_pin = { kGPIO_DigitalOutput, 0 };
	/* Configure GPIO */
	GPIO_PinInit(GPIOC, MATRIX_POWER_PIN, &matrix_power_pin);
	GPIO_PortSet(GPIOC, 1u << MATRIX_POWER_PIN);

	/* Waste some time. */
	volatile uint32_t t = 0xFFFFFF;
	while (t-- != 0);
	matrix_init();

	vTaskPrioritySet(NULL, tskIDLE_PRIORITY + 3);
	while (1) {
		EventBits_t eb = xEventGroupWaitBits(matrix_ev_group, MATRIX_EVENT_SEND, pdTRUE, pdFALSE, portMAX_DELAY);
		if (eb & MATRIX_EVENT_SEND){
			matrix_print();
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
