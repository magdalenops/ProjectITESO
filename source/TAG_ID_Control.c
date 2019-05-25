/*
 * TAG_ID_Control.c
 *
 *  Created on: 5 mar. 2019
 *      Author: Dell
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
/* Includes for tag id validation module ------------------------------------------------------------------------------------------------------------------------------- */
/*#include "Stdint.h"
#include "string.h"
#include "FreeRTOS.h"
#include "fsl_debug_console.h"
#include "semphr.h"
#include "task.h"*/

/*******************************************************************************
 * Includes
 ******************************************************************************/

    /* INCLUDES      for leds and buttons   ----------------------------------------------------------------- FINAL PROYECT*/
//for GPIOS Buttons
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_debug_console.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_common.h"
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
//For semaphores
/*#include "queue.h"
#include "timers.h"
#include "semphr.h"*/
    /*                  ----------------------------------------------------------------- FINAL PROYECT*/



/* Includes for flash memory module ------------------------------------------------------------------------------------------------------------------------------- */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "clock_config.h"
#include "fsl_flash1.h"
#if defined(FSL_FEATURE_HAS_L1CACHE) && FSL_FEATURE_HAS_L1CACHE
#include "fsl_cache.h"
#endif /* FSL_FEATURE_HAS_L1CACHE */
#include "pin_mux.h"


/* Code from flash memory ----------------------------------------------------------------------------------------------------------------------------------------- */
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BUFFER_LEN 4

/* Code from tag validation ----------------------------------------------------------------------------------------------------------------------------------------- */
#define AUTHORIZE 	0x01
#define APPROVED	0x02
#define DENIED		0x03

/*******************************************************************************
 * Variables
 ******************************************************************************/


    /*    DEFINES     for leds and buttons         ----------------------------------------------------------------- FINAL PROYECT*/
//RED LED
#define BOARD_LED_GPIO BOARD_LED_RED_GPIO
#define BOARD_LED_GPIO_PIN BOARD_LED_RED_GPIO_PIN
//BLUE LED
#define BOARD_LED_GPIO_B BOARD_LED_BLUE_GPIO
#define BOARD_LED_GPIO_PIN_B BOARD_LED_BLUE_GPIO_PIN
//GREEN LED
#define BOARD_LED_GPIO_G BOARD_LED_GREEN_GPIO
#define BOARD_LED_GPIO_PIN_G BOARD_LED_GREEN_GPIO_PIN

#define BOARD_SW_GPIO BOARD_SW3_GPIO
#define BOARD_SW_PORT BOARD_SW3_PORT
#define BOARD_SW_GPIO_PIN BOARD_SW3_GPIO_PIN
#define BOARD_SW_IRQ BOARD_SW3_IRQ
#define BOARD_SW_IRQ_HANDLER BOARD_SW3_IRQ_HANDLER
#define BOARD_SW_NAME BOARD_SW3_NAME
 /*                  ----------------------------------------------------------------- FINAL PROYECT*/



/* Code from flash memory ----------------------------------------------------------------------------------------------------------------------------------------- */

/*! @brief Flash driver Structure */
static flash_config_t_f s_flashDriver;
/*! @brief Flash cache driver Structure */
static ftfx_cache_config_t s_cacheDriver;
/*! @brief Buffer for program */
static uint32_t s_buffer[BUFFER_LEN];
/*! @brief Buffer for readback */
static uint32_t s_buffer_rbc[BUFFER_LEN];
status_t result;    /* Return code from each flash driver function */
uint32_t destAdrss; /* Address of the target location */
uint32_t i, failAddr, failDat;
uint32_t pflashBlockBase = 0;
uint32_t pflashTotalSize = 0;
uint32_t pflashSectorSize = 0;


/* Code from tag validation ----------------------------------------------------------------------------------------------------------------------------------------- */
/* Global variables */
/* To save the arrangement of the ID*/
uint8_t TAG[11];
/* To save the ID in a single variable*/
uint64_t TAG_ID;
/* To store the ID to compare and the one save by the user*/
uint32_t Data_Base[3];
/* To know the status of the mode, delete or add ID*/
uint8_t modeStatus = 0x00;
uint8_t SaveFlashIDStatus = 0x00;
/* To know status if can be save the ID*/
uint8_t IDStatus;
uint8_t IDStatusControl;
/* Flag to ensure that NDEF message can be send*/
uint8_t NDEFMessageStatus;
uint8_t NDEFMessageFlag;
uint8_t NDEFMessageFlag2;


/*******************************************************************************
 * Code
 ******************************************************************************/

void delay(void)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 2999999; ++i)
    {
        __asm("NOP"); /* delay */
    }
}

void vfcnToggleLed (uint8_t Color)
{

	/* Table of colors
	 *
	 * 1 for RED
	 * 2 FOR BLUE
	 * 3 FOR GREEN
	 * 4 FOR WHITE
	 *
	 */

	  switch ( Color )
	  {
	  case 1:

		/* Toggle RED LED ON. */
		GPIO_TogglePinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_PIN);
		delay();delay();delay();
		/* Toggle RED LED OFF. */
		GPIO_TogglePinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_PIN);
	    break;

	  case 2:
		/* Toggle BLUE LED ON. */
		GPIO_TogglePinsOutput(BOARD_LED_GPIO_B, 1U << BOARD_LED_GPIO_PIN_B);
		delay();delay();delay();
		/* Toggle BLUE LED OFF. */
		GPIO_TogglePinsOutput(BOARD_LED_GPIO_B, 1U << BOARD_LED_GPIO_PIN_B);
	    break;

	  case 3:
		/* Toggle GREEN LED ON. */
		GPIO_TogglePinsOutput(BOARD_LED_GPIO_G, 1U << BOARD_LED_GPIO_PIN_G);
		delay();delay();delay();
		/* Toggle GREEN LED OFF. */
		GPIO_TogglePinsOutput(BOARD_LED_GPIO_G, 1U << BOARD_LED_GPIO_PIN_G);
	    break;

	  case 4:
		/* Toggle WHITE LED ON. */
		GPIO_TogglePinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_PIN);
		GPIO_TogglePinsOutput(BOARD_LED_GPIO_B, 1U << BOARD_LED_GPIO_PIN_B);
		GPIO_TogglePinsOutput(BOARD_LED_GPIO_G, 1U << BOARD_LED_GPIO_PIN_G);
		delay();delay();delay();
		/* Toggle WHITE LED OFF. */
		GPIO_TogglePinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_PIN);
		GPIO_TogglePinsOutput(BOARD_LED_GPIO_B, 1U << BOARD_LED_GPIO_PIN_B);
		GPIO_TogglePinsOutput(BOARD_LED_GPIO_G, 1U << BOARD_LED_GPIO_PIN_G);
	    break;

	  default:
	    PRINTF(" No valid parameter set \r\n");
	    break;
	  }

}


/* Code from flash memory ----------------------------------------------------------------------------------------------------------------------------------------- */

/*
* @brief Gets called when an error occurs.
* @details Print error message and trap forever.
*/
void error_trap(void)
{
    PRINTF("\r\n\r\n\r\n\t---- HALTED DUE TO FLASH ERROR! ----");
    while (1)
    {
    }
}

/*
* @brief Initialization code for flash memory.
* @details .
*/
void flash_init(void)
{
	ftfx_security_state_t securityStatus = kFTFx_SecurityStateNotSecure; /* Return protection status */

    /* Init hardware */
    /*BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();*/

    /* Clean up Flash, Cache driver Structure*/
    memset(&s_flashDriver, 0, sizeof(flash_config_t_f));
    memset(&s_cacheDriver, 0, sizeof(ftfx_cache_config_t));

    /* Setup flash driver structure for device and initialize variables. */
    result = FLASH_Init_f(&s_flashDriver);
    if (kStatus_FTFx_Success != result)
    {
        error_trap();
    }
    /* Setup flash cache driver structure for device and initialize variables. */
    result = FTFx_CACHE_Init(&s_cacheDriver);
    if (kStatus_FTFx_Success != result)
    {
        error_trap();
    }
    /* Get flash properties*/
    FLASH_GetProperty_f(&s_flashDriver, kFLASH_PropertyPflash0BlockBaseAddr, &pflashBlockBase);
    FLASH_GetProperty_f(&s_flashDriver, kFLASH_PropertyPflash0TotalSize, &pflashTotalSize);
    FLASH_GetProperty_f(&s_flashDriver, kFLASH_PropertyPflash0SectorSize, &pflashSectorSize);

    /* print welcome message */
    PRINTF("\r\n PFlash Example Start \r\n");
    /* Print flash information - PFlash. */
    PRINTF("\r\n PFlash Information: ");
    PRINTF("\r\n Total Program Flash Size:\t%d KB, Hex: (0x%x)", (pflashTotalSize / 1024), pflashTotalSize);
    PRINTF("\r\n Program Flash Sector Size:\t%d KB, Hex: (0x%x) ", (pflashSectorSize / 1024), pflashSectorSize);

    /* Check security status. */
    result = FLASH_GetSecurityState_f(&s_flashDriver, &securityStatus);
    if (kStatus_FTFx_Success != result)
    {
        error_trap();
    }
    /* Print security status. */
    switch (securityStatus)
    {
        case kFTFx_SecurityStateNotSecure:
            PRINTF("\r\n Flash is UNSECURE!");
            break;
        case kFTFx_SecurityStateBackdoorEnabled:
            PRINTF("\r\n Flash is SECURE, BACKDOOR is ENABLED!");
            break;
        case kFTFx_SecurityStateBackdoorDisabled:
            PRINTF("\r\n Flash is SECURE, BACKDOOR is DISABLED!");
            break;
        default:
            break;
    }
    PRINTF("\r\n");

    /* Test pflash basic opeation only if flash is unsecure. */
    if (kFTFx_SecurityStateNotSecure == securityStatus)
    {
        /* Pre-preparation work about flash Cache/Prefetch/Speculation. */
        FTFx_CACHE_ClearCachePrefetchSpeculation(&s_cacheDriver, true);

        /* Debug message for user. */
        /* Erase several sectors on upper pflash block where there is no code */
        PRINTF("\r\n Erase a sector of flash");


/* In case of the protected sectors at the end of the pFlash just select
the block from the end of pFlash to be used for operations
SECTOR_INDEX_FROM_END = 1 means the last sector,
SECTOR_INDEX_FROM_END = 2 means (the last sector - 1) ...
in case of FSL_FEATURE_FLASH_HAS_PFLASH_BLOCK_SWAP it is
SECTOR_INDEX_FROM_END = 1 means the last 2 sectors with width of 2 sectors,
SECTOR_INDEX_FROM_END = 2 means the last 4 sectors back
with width of 2 sectors ...
*/
#ifndef SECTOR_INDEX_FROM_END
  #define SECTOR_INDEX_FROM_END 1U
#endif

/* Erase a sector from destAdrss. */
#if defined(FSL_FEATURE_FLASH_HAS_PFLASH_BLOCK_SWAP) && FSL_FEATURE_FLASH_HAS_PFLASH_BLOCK_SWAP
        /* Note: we should make sure that the sector shouldn't be swap indicator sector*/
        destAdrss = pflashBlockBase + (pflashTotalSize - (SECTOR_INDEX_FROM_END * pflashSectorSize * 2));
#else
        destAdrss = pflashBlockBase + (pflashTotalSize - (SECTOR_INDEX_FROM_END * pflashSectorSize));
#endif

        result = FLASH_Erase_f(&s_flashDriver, destAdrss, pflashSectorSize, kFTFx_ApiEraseKey);
        if (kStatus_FTFx_Success != result)
        {
            error_trap();
        }

        /* Verify sector if it's been erased. */
        result = FLASH_VerifyErase_f(&s_flashDriver, destAdrss, pflashSectorSize, kFTFx_MarginValueUser);
        if (kStatus_FTFx_Success != result)
        {
            error_trap();
        }

        /* Print message for user. */
        PRINTF("\r\n Successfully Erased Sector 0x%x -> 0x%x\r\n", destAdrss, (destAdrss + pflashSectorSize));

        /* Print message for user. */
        PRINTF("\r\n Program a buffer to a sector of flash ");
        /* Prepare user buffer. */
       /* for (i = 0; i < BUFFER_LEN; i++)
        {
            s_buffer[i] = i;
        }*/
        //s_buffer[0]= 0x32;
	}
	else
    {
        PRINTF("\r\n Erase/Program opeation will not be executed, as Flash is SECURE!");
    }
}

void flash_write (void)
{

// next step after initialization of flash
        /* Program user buffer into flash*/
        result = FLASH_Program_f(&s_flashDriver, destAdrss, (uint8_t *)s_buffer, sizeof(s_buffer));
        if (kStatus_FTFx_Success != result)
        {
            error_trap();
        }

        /* Verify programming by Program Check command with user margin levels */
        result = FLASH_VerifyProgram_f(&s_flashDriver, destAdrss, sizeof(s_buffer), (const uint8_t *)s_buffer, kFTFx_MarginValueUser,
                                     &failAddr, &failDat);
        if (kStatus_FTFx_Success != result)
        {
            error_trap();
        }

        /* Post-preparation work about flash Cache/Prefetch/Speculation. */
        FTFx_CACHE_ClearCachePrefetchSpeculation(&s_cacheDriver, false);

#if defined(FSL_FEATURE_HAS_L1CACHE) && FSL_FEATURE_HAS_L1CACHE
        L1CACHE_InvalidateCodeCache();
#endif /* FSL_FEATURE_HAS_L1CACHE */

#if defined(__DCACHE_PRESENT) && __DCACHE_PRESENT
        /* Clean the D-Cache before reading the flash data*/
        SCB_CleanInvalidateDCache();
#endif
        /* Verify programming by reading back from flash directly*/
        for (uint32_t i = 0; i < BUFFER_LEN; i++)
        {
            s_buffer_rbc[i] = *(volatile uint32_t *)(destAdrss + i * 4);
            if (s_buffer_rbc[i] != s_buffer[i])
            {
                error_trap();
            }
        }

        PRINTF("\r\n Successfully Programmed and Verified Location 0x%x -> 0x%x \r\n", destAdrss,
               (destAdrss + sizeof(s_buffer)));

        /* Erase the context we have progeammed before*/
        /* Note: we should make sure that the sector which will be set as swap indicator should be blank*/
        //FLASH_Erase(&s_flashDriver, destAdrss, pflashSectorSize, kFTFx_ApiEraseKey);



}


void flash_read (void)
{

	        /* Reading back from flash directly*/
        for (uint32_t i = 0; i < 2; i++)
        {
            s_buffer_rbc[i] = *(volatile uint32_t *)(destAdrss + i * 4);
            //if (s_buffer_rbc[i] != s_buffer[i])
            if (s_buffer_rbc[i] != Data_Base[i])
            {
            	NDEFMessageStatus = DENIED;
            	vfcnToggleLed(1);
            	break;
                //error_trap();
            }
            else
            {
            	NDEFMessageStatus = APPROVED;
            }
        }

            if(NDEFMessageStatus == APPROVED)
            {
            		/* FLAG TO UNLOCK THE LOCK AND START THE TRIP*/
            	if(NDEFMessageFlag2 == 0)
            	{
            		vfcnToggleLed(3);
            		NDEFMessageFlag = APPROVED;
            	}
            		NDEFMessageFlag2++;

            	if(NDEFMessageFlag2 == 0x02)
            	{
            		NDEFMessageFlag2 = 0x00;
            		/* FLAG TO FINISH TRIP AND SEND DATA TO TAG */
					NDEFMessageFlag = AUTHORIZE;
            	}

            }

}

void flash_delete (void)
{
	FLASH_Erase_f(&s_flashDriver, destAdrss, pflashSectorSize, kFTFx_ApiEraseKey);
}










void vfcnSaveID (unsigned char * ID)
{

	uint8_t lbControl;

	for (lbControl = 0; lbControl < 7; lbControl++)
	{
		TAG[lbControl]=*ID;
		ID++;
	}

	TAG_ID = TAG[0] << 8;
	TAG_ID = TAG_ID + TAG[1];
	TAG_ID = TAG_ID << 8;
	TAG_ID = TAG_ID + TAG[2];
	TAG_ID = TAG_ID << 8;
	TAG_ID = TAG_ID + TAG[3];
	Data_Base[0] = TAG_ID;



	TAG_ID = TAG[4] << 8;
	TAG_ID = TAG_ID + TAG[5];
	TAG_ID = TAG_ID << 8;
	TAG_ID = TAG_ID + TAG[6];
	TAG_ID = TAG_ID << 8;
	TAG_ID = TAG_ID + TAG[7];
	Data_Base[1] = TAG_ID;

    if(SaveFlashIDStatus == 0x01)
    {
		for (lbControl = 0; lbControl < 2; lbControl++)
		{
			s_buffer[lbControl] = Data_Base[lbControl];
		}
		SaveFlashIDStatus = 0x00;
		flash_write();
		PRINTF(" ID registered successfully \r\n");
    }


	/*if(IDStatus == OK && IDStatusControl == APPROVED)
	{
		Data_Base[0] = TAG_ID;
		PRINTF(" ID registered successfully \r\n");
		IDStatus = 0x00;
		IDStatusControl = 0x01;

	}*/


}


void vfcndeleteID (void)
{
	flash_delete();
	PRINTF("ID Erased from memory\r\n");
}


void vfcnValidateID (void)
{


	/* Check if buffer for flash has saved an ID */
	if(s_buffer[0] != 0 && s_buffer[1] != 0)
	{
	flash_read();
	}
	else
	{
		vfcnToggleLed(1);
		NDEFMessageStatus = DENIED;
		PRINTF(" No TAG has been registered yet \r\n");
		PRINTF(" You need to register a TAG in order to proceed \r\n");
		modeStatus = 0x00;
	}

	/*if(Data_Base[0] == Data_Base[1])
	{
		NDEFMessageStatus = 0x01;
	}
	else
	{
		NDEFMessageStatus = 0x00;
	}*/
}








/* section for buttons and leds  */

void GPIOinit(void)
{

	/*         INIT         ----------------------------------------------------------------- FINAL PROYECT*/
/* Define the init structure for the input switch pin*/
gpio_pin_config_t sw_config = {
	kGPIO_DigitalInput, 0,
};

/* Define the init structure for the output LED pin*/
gpio_pin_config_t led_config = {
	kGPIO_DigitalOutput, 0,
};
/*                               ----------------------------------------------------------------- FINAL PROYECT*/


/* INIT FOR BUTTONS                ----------------------------------------------------------------- FINAL PROYECT*/
    /* Init input switch GPIO. SW3*/
    PORT_SetPinInterruptConfig(BOARD_SW3_PORT, BOARD_SW3_GPIO_PIN, kPORT_InterruptFallingEdge);
    EnableIRQ(BOARD_SW3_IRQ);
    GPIO_PinInit(BOARD_SW3_GPIO, BOARD_SW3_GPIO_PIN, &sw_config);
    /* Init input switch GPIO. SW2*/
    PORT_SetPinInterruptConfig(BOARD_SW2_PORT, BOARD_SW2_GPIO_PIN, kPORT_InterruptFallingEdge);
    EnableIRQ(BOARD_SW2_IRQ);
    GPIO_PinInit(BOARD_SW2_GPIO, BOARD_SW2_GPIO_PIN, &sw_config);

    /* Init output LED GPIO.  RED*/
    GPIO_PinInit(BOARD_LED_GPIO, BOARD_LED_GPIO_PIN, &led_config);
	/* Toggle RED LED OFF. RED*/
	GPIO_TogglePinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_PIN);
    /* Init output LED GPIO. BLUE*/
    GPIO_PinInit(BOARD_LED_GPIO_B, BOARD_LED_GPIO_PIN_B, &led_config);
	/* Toggle BLUE LED OFF. BLUE*/
	GPIO_TogglePinsOutput(BOARD_LED_GPIO_B, 1U << BOARD_LED_GPIO_PIN_B);
    /* Init output LED GPIO. GREEN*/
    GPIO_PinInit(BOARD_LED_GPIO_G, BOARD_LED_GPIO_PIN_G, &led_config);
	/* Toggle GREEN LED OFF. GREEN*/
	GPIO_TogglePinsOutput(BOARD_LED_GPIO_G, 1U << BOARD_LED_GPIO_PIN_G);

	/*SET PORTA INTERRUPT LEVEL TO 3, configMAX_SYSCALL_INTERRUPT_PRIORITY priority is 2*/
	NVIC_SetPriority(59,3);/*PortA vector is 59*/
	NVIC_SetPriority(61,3);/*PortC vector is 61*/
    /* FINALE                       ----------------------------------------------------------------- FINAL PROYECT*/

}




/* Interrupt for button ----------------------------------------------------------------- FINAL PROYECT*/
void BOARD_SW3_IRQ_HANDLER(void)
{
	GPIO_ClearPinsInterruptFlags(BOARD_SW_GPIO, 1U << BOARD_SW_GPIO_PIN);
	vfcnToggleLed(2);
	SaveFlashIDStatus = 0x01;
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
/*                      ----------------------------------------------------------------- FINAL PROYECT*/
