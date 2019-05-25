/*
 * gps.c
 *
 *  Created on: 21/04/2019
 *  Author: Ivan Gutierrez
 *  brief:	Configuration and control module for gps (include uart 4)
 */


#include "gps.h"	//Own header
#include "math.h"	//Math library

/* Pin configuration */
#define PIN14_IDX						14u		//Pin number for 14 in a port
#define PIN15_IDX						15u		//Pin number for 15 in a port
#define SOPT5_UART0TXSRC_UART_TX      0x00u   /*!< UART 0 transmit data source select: UART0_TX pin */
#define SOPT5_UART4TXSRC_UART_TX	  0x00u

/* Macros */
#define GPS_MESSAGE_BUFFER_SIZE		67	//Max size of GPS RMC frame


#define LAT_SIZE					9u	//Latitude size in GPS buffer
#define VALID_FRAME_POSITION		15u	//Position in the GPS frame to know if it is a valid frame , in this position should be a 'A' if it is a valid frame
#define INIT_LAT_POS				17u	//Initial position of latitude
#define LON_SIZE					10u	//Longitude size in GPS buffer
#define LAT_HEMISPHERE				28u	//N or S
#define INIT_LON_POS				30u	//Initial position of longitude
#define LON_HEMISPHERE				42u	//W or O



const uint8_t g_tipString[] =
    "Uart functional API interrupt example\r\nBoard receives characters then sends them out\r\nNow please input:\r\n";

volatile uint16_t txIndex; /* Index of the data to send out. */
volatile uint16_t rxIndex; /* Index of the memory to save new arrived data. */

volatile uint8_t gpsMessageBuffer[GPS_MESSAGE_BUFFER_SIZE];	//GPS buffer to save data

volatile float flCurrentLatitude = 0;
volatile float flOldLatitude = 0;
volatile float flCurrentLongitude = 0;
volatile float flOldLongitude = 0;
volatile unsigned char gbDate = 0;
volatile unsigned char gbMon = 0;
volatile unsigned char gbYear = 0;
volatile unsigned char gbHour = 0;
volatile unsigned char gbMin = 0;
volatile unsigned char gbSec = 0;

/********************* Private functions *********************/

/*************************************************************************************************************/
//! @fn		float ffnGetLat(void)
//! @brief	Get latitude from GPS.
//! @return Latitude in floating point (4 bytes) - Deegres
//! @author Ivan Gutierrez
/*************************************************************************************************************/
static float ffnGetLat(void)
{
	unsigned char lbaLatitudeGPS[LAT_SIZE];		///< To catch bytes
	unsigned char lbControl;
	unsigned char lbFrameDeegres = 0;			///< Store deegres of the incoming frame
	float lfDeegres = 0;						///< Store latitude in deegres


	for(lbControl = 0 ; lbControl < LAT_SIZE ; lbControl++)
	{
		lbaLatitudeGPS[lbControl] = gpsMessageBuffer[INIT_LAT_POS + lbControl];		// Store ascii number
		lbaLatitudeGPS[lbControl] -= 0x30;															// Convert to decimal
	}

	lbFrameDeegres = lbaLatitudeGPS[0] * 10 + lbaLatitudeGPS[1];		// Store frame deegres


	// Minutes to deegres
	lfDeegres = (lbaLatitudeGPS[2] * 100000 + lbaLatitudeGPS[3] * 10000 + lbaLatitudeGPS[5] * 1000
				+ lbaLatitudeGPS[6] * 100 + lbaLatitudeGPS[7] * 10 + lbaLatitudeGPS[8]) ;
	lfDeegres /= 600000;
	lfDeegres += lbFrameDeegres;	// Add deegres from incoming frame

	if(gpsMessageBuffer[LAT_HEMISPHERE] == 'S')	lfDeegres *= -1;	// Change sign if it is necessary

	return lfDeegres;
}





/*************************************************************************************************************/
//! @fn		float ffnGetLon(void)
//! @brief	Get longitude from GPS.
//! @return Longitude in floating point (4 bytes) - Deegres
//! @author Ivan Gutierrez
/*************************************************************************************************************/
static float ffnGetLon(void)
{
	unsigned char lbaLonGPS[10];
	unsigned char lbControl;
	int lwFrameDeegres = 0;		///< Store deegres of the incoming frame
	float lfDeegres = 0;			///< Store longitude in deegres


	for(lbControl = 0 ; lbControl < LON_SIZE ; lbControl++)
	{
		lbaLonGPS[lbControl] = gpsMessageBuffer[INIT_LON_POS + lbControl];
		lbaLonGPS[lbControl] -= 0x30;								// Convert to decimal
	}

	lwFrameDeegres = lbaLonGPS[0] * 100 + lbaLonGPS[1] * 10 + lbaLonGPS[2];		// Store frame deegres

	// Minutes to deegres
	//lfDeegres = (float)((float)(lbaLonGPS[3] * 100000) + ((float)(lbaLonGPS[4] * 10000)) + (float)(lbaLonGPS[6] * 1000) + (float)(lbaLonGPS[7] * 100) + (float)(lbaLonGPS[8] * 10) + (float)(lbaLonGPS[9]));


	/* Make this way to prevent compiler issue */
	int ldwTemp = 0;
    ldwTemp = (int)lbaLonGPS[4] * 10000;

	lfDeegres = lbaLonGPS[3] * 100000;
	lfDeegres += ldwTemp;
	lfDeegres += lbaLonGPS[6] * 1000;
	lfDeegres += lbaLonGPS[7] * 100;
	lfDeegres += lbaLonGPS[8] * 10;
	lfDeegres += lbaLonGPS[9];


	lfDeegres /= 600000;
	lfDeegres += lwFrameDeegres;	// Add deegres from incoming frame

	if(gpsMessageBuffer[LON_HEMISPHERE] == 'W')	lfDeegres *= -1;	// Change sign

	return lfDeegres;
}


/*************************************************************************************************************/
//! @fn		BYTE bfnZeller(int anno, int mes, int dia)
//! @parm	Year, month and day to determinate the day of the week.
//! @brief	Determinate the day of the week of the param using Zeller's Congruence.
//! @return	Day of the week.
/*************************************************************************************************************/
static unsigned char bfnZeller(unsigned char anno, unsigned char mes, unsigned char dia)
{
    if (mes <= 2)
    {
      mes = mes + 10;
      anno = anno - 1;
    }
    else
    {
      mes = mes - 2;
    }

    int a = anno % 100;
    int b = anno / 100;

    int resultado =
    (
      700 +
      ((26 * mes - 2) / 10) +
      dia +
      a +
      a / 4 +
      b / 4 -
      2 * b
    ) % 7;

    return resultado;
}


/*************************************************************************************************************/
//! @fn		void vfnSetUTCOffset(void)
//! @brief	Set UTC offset to incoming date and hour.
//! @retunr Offset in seconds
//! @author Ivan Gutierrez
/*************************************************************************************************************/
static int vfnSetUTCOffset(void)
{
	unsigned int lbSummerDay, lbWinterDay;
	unsigned int lwSetOffset = 0;		// Offset in seconds

	/* Get days limits to offset */
	lbSummerDay = 8 - bfnZeller(gbYear, 4, 1);			// Get first april sunday
	if(lbSummerDay == 8)	lbSummerDay = 1;
	lbWinterDay = 31 - bfnZeller(gbYear, 10, 31);		// Get last october sunday


	switch(gbMon)
	{
		case 1:			// January
			lwSetOffset = 21600;		// 6 hours in seconds
		break;


		case 2:			// Frebuary
			lwSetOffset = 21600;		// 6 hours in seconds
		break;


		case 3:			// March
			lwSetOffset = 21600;		// 6 hours in seconds
		break;

		case 4:			// April

			if(gbDate < lbSummerDay)
				lwSetOffset = 21600;		// 6 hours in seconds


			if(gbDate == lbSummerDay && gbHour < 2)
				lwSetOffset = 21600;		// 6 hours in seconds


			if(gbDate == lbSummerDay && gbHour >= 2)
				lwSetOffset = 18000;		// 5 hours in seconds

			if(gbDate > lbSummerDay)
				lwSetOffset = 18000;		// 5 hours in seconds
		break;


		case 5:			// May
			lwSetOffset = 18000;		// 5 hours in seconds
		break;


		case 6:			// June
			lwSetOffset = 18000;		// 5 hours in seconds
		break;

		case 7:			// July
			lwSetOffset = 18000;		// 5 hours in seconds
		break;


		case 8:			// August
			lwSetOffset = 18000;		// 5 hours in seconds
		break;


		case 9:			// September
			lwSetOffset = 18000;		// 5 hours in seconds
		break;



		case 10:		// October

			if(gbDate < lbWinterDay)
				lwSetOffset = 18000;		// 5 hours in seconds

			if(gbDate == lbWinterDay && gbHour < 2)
				lwSetOffset = 18000;		// 5 hours in seconds

			if(gbDate == lbWinterDay && gbHour >= 2)
				lwSetOffset = 21600;		// 6 hours in seconds

			if(gbDate > lbWinterDay)
				lwSetOffset = 21600;		// 6 hours in seconds
		break;

		case 11:		// November
			lwSetOffset = 21600;		// 6 hours in seconds
		break;

		case 12:		// December
			lwSetOffset = 21600;		// 6 hours in seconds
		break;

	}

	return lwSetOffset;

}


/*
 *  \fn:			static float flfnCalculateDistance(void)
 *  \brief:			Calculates the distance in meters between two geographical points without considering the curvature of the earth
 *  \param (in): 	void
 *  \param (out):	void
 *  \return:		float: Distance in meters
 *  \author:		Ivan Gutierrez
 *   */
static float flfnCalculateDistance(void)
{
	float lflResult = 0, lflPot1 = 0, lflPot2 = 0;

	if((flOldLatitude == 0 || flOldLongitude == 0 || flCurrentLatitude == 0 || flCurrentLongitude == 0) ||
		(flOldLatitude > 50 || (flOldLongitude * -1) > 200 || flCurrentLatitude > 50 || (flCurrentLongitude * -1) > 200))	//If there is no information to complete the calculation
	{
		return lflResult;	//Return 0
	}

	lflPot1 = (pow(((flOldLatitude) - (flCurrentLatitude)), 2));
	lflPot2 = (pow(((flOldLongitude) - (flCurrentLongitude)), 2));

	lflResult = (sqrt( lflPot1 + lflPot2 )) * 10000;

	//lflResult = (sqrt( (pow(((flOldLatitude) - (flCurrentLatitude)), 2)) + (pow(((flOldLongitude) - (flCurrentLongitude)), 2)) )) * 100000;

	if(lflResult > 2.3)		//If distance > 2.3m, then is a valid distance, less than 2.3m maybe it is a GPS error
	{
		return lflResult;	//Return a valid distance
	}else
	{
		return 0;			//Return 0 if distance is less than 2.3m, because maybe it is a GPS error
	}

}



/*************************************************************************************************************/
//! @fn		void vfnDateInSeconds(void)
//! @brief	Determinate date and hour in seconds since 1980.
//! @param	Offset in secods
//! @return Number of seconds
//! @author Ivan Gutierrez
/*************************************************************************************************************/
static unsigned int dwfnDateInSeconds(unsigned int lwSetOffset)
{
//Number of days since January 01, 1980
//
unsigned int NoDeDiasTotal = 0;				//Start in 2000 -> 2000-1980 = 20
unsigned char NoDeAniosBisiestos = 0;
unsigned int DiasAccxAnio[13] = {0, 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

unsigned int ldwSeconds = 0;						///< To save date and hour in seconds

	NoDeAniosBisiestos = ((20 + gbYear) / 4);
	NoDeDiasTotal = (365 * (20 + gbYear)) + NoDeAniosBisiestos;
	NoDeDiasTotal += DiasAccxAnio[gbMon]; 							// Acumulado x Meses

	if( (gbYear % 4) == 0  && (gbMon > 2))				// años bisiestos +/- 1
		NoDeDiasTotal += 1;

	NoDeDiasTotal += gbDate;					  						// Dias del Mes

//***************************
// Restamos uno por que al hacer este calculo estamos tomando en cuenta
// 2 veces el primero de Enero. ( una al Sacar la diferencia desde
// 1 Enero 1980 a 1 Enero 20XX(actual) y otra al sumar
// de 1 enero 20xx(actual) hasta hoy.

	//NoDeDiasTotal++;						///< Maybe it is not necessary

	unsigned int ldwTemp = 0, ldwTemp1 = 0, ldwTemp2 = 0, ldwTemp3 = 0;

    ldwTemp = (unsigned int)NoDeDiasTotal * 86400;		///< Seconds of finished days since 1980
	ldwTemp1 = (unsigned int)gbHour * 3600;
	ldwTemp2 = (unsigned int)gbMin * 60 ;
	ldwTemp3 = (unsigned int)gbSec;						///< Total seconds

	ldwSeconds = ldwTemp + ldwTemp1 +ldwTemp2 + ldwTemp3;

	ldwSeconds -= lwSetOffset;

	return ldwSeconds;	// 1224873000  -  1224891000
}



/*
 *  \fn:			void vfnInitGPS_UART(void)
 *  \brief:			Get velocity in km/h
 *  \param (in): 	void
 *  \param (out):	void
 *  \return:		Km/h (float)
 *  \author:		Ivan Gutierrez
 *   */
static float flfnGetVel(void)
{
	float lflSpeedKnots = 0;
	float lflSpeedKm = 0;

	lflSpeedKnots += (gpsMessageBuffer[46] - 0x30) * 100;
	lflSpeedKnots += (gpsMessageBuffer[47] - 0x30) * 10;
	lflSpeedKnots += (gpsMessageBuffer[48] - 0x30);
	lflSpeedKnots /= 1000;
	lflSpeedKnots += (gpsMessageBuffer[44] - 0x30);
	lflSpeedKm = lflSpeedKnots * 1.852;

	if(lflSpeedKm > 3)
		return lflSpeedKm;
	else
		return 0;
}



/********************* Public functions *********************/

/*
 *  \fn:			void vfnInitGPS_UART(void)
 *  \brief:			Initialize GPS UART and enable Rx interrupt (this is a public function)
 *  \param (in): 	void
 *  \param (out):	void
 *  \return:		void
 *  \author:		Ivan Gutierrez
 *   */
void vfnInitGPS_UART(void)
{
	/* Port clock and pin configurtion */
	  //CLOCK_EnableClock(kCLOCK_PortC);                           /* Port C Clock Gate Control: Clock enabled - UART4 */

	  PORT_SetPinMux(PORTC, PIN14_IDX, kPORT_MuxAlt3);           /* PORTB16 (pin 86) is configured as UART4_RX */
	  PORT_SetPinMux(PORTC, PIN15_IDX, kPORT_MuxAlt3);           /* PORTB17 (pin 87) is configured as UART4_TX */

	uart_config_t config;
    /*
     * config.baudRate_Bps = 9600u;
     * config.parityMode = kUART_ParityDisabled;
     * config.stopBitCount = kUART_OneStopBit;
     * config.txFifoWatermark = 0;
     * config.rxFifoWatermark = 1;
     * config.enableTx = false;
     * config.enableRx = false;
     */
    UART_GetDefaultConfig(&config);
    config.baudRate_Bps = GPS_BAUDRATE;
    config.enableTx = true;
    config.enableRx = true;

    UART_Init(GPS_UART, &config, GPS_UART_CLK_FREQ);

    /* Send g_tipString out. */
    UART_WriteBlocking(GPS_UART, g_tipString, sizeof(g_tipString) / sizeof(g_tipString[0]));

    /* Enable RX interrupt. */
    NVIC_SetPriority(GPS_UART_IRQn, 5);
    UART_EnableInterrupts(GPS_UART, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable);
    EnableIRQ(GPS_UART_IRQn);
}



/*
 *  \fn:			void vfnGetDataGPS(unsigned char * lbaDataGPS)
 *  \brief:			Initializes GPS UART (public function), this is a critical zone of code,
 *  				because gpsMessageBuffer is a critical resource that is modified in UART Rx interrupt
 *  \param (in): 	void
 *  \param (out):	lbaDataGPS: To save the current GPS data
 *  \return:		void
 *  \author:		Ivan Gutierrez
 *   */
unsigned char vfnGetDataGPS(stGPSData_t * stDataGPS)
{
	DisableIRQ(GPS_UART_IRQn);				//Disable UART 4 (GPS) interrupts
	float lflDistance = 0, lflSpeed = 0;
	unsigned int ldwDateinSeconds = 0;

	/* RMC frame validation */
	if(gpsMessageBuffer[VALID_FRAME_POSITION] == 'V')	//If it is a corrupt frame, return GPS_DATA_NO_OK
	{
		EnableIRQ(GPS_UART_IRQn);		//Enable UART 4 (GPS) interrupts
		return GPS_DATA_NO_OK;
	}



	if(gpsMessageBuffer[VALID_FRAME_POSITION] == 'A')	//If it is a valid frame, process data
	{
		/* Get current latitude and longitude */
		flCurrentLatitude = ffnGetLat();		//Get new latitude
		flCurrentLongitude = ffnGetLon();		//Get new longitude
		stDataGPS -> Latitude = flCurrentLatitude;
		stDataGPS -> Longitude = flCurrentLongitude;
		lflDistance = flfnCalculateDistance();	//Get current distance
		stDataGPS -> Distance = lflDistance;
		flOldLatitude = flCurrentLatitude;		//Load old latitude
		flOldLongitude = flCurrentLongitude;	//Load old longitude


		/* Get current time data */
		gbDate = ((gpsMessageBuffer[51] - 0x30) * 10) + (gpsMessageBuffer[52] - 0x30);
		gbMon = ((gpsMessageBuffer[53] - 0x30) * 10) + (gpsMessageBuffer[54] - 0x30);
		gbYear = ((gpsMessageBuffer[55] - 0x30) * 10) + (gpsMessageBuffer[56] - 0x30);

		gbHour = ((gpsMessageBuffer[5]- 0x30) * 10) + (gpsMessageBuffer[6] - 0x30);
		gbMin = ((gpsMessageBuffer[7] - 0x30) * 10) + (gpsMessageBuffer[8] - 0x30);
		gbSec = ((gpsMessageBuffer[9] - 0x30) * 10) + (gpsMessageBuffer[10] - 0x30);

		ldwDateinSeconds = dwfnDateInSeconds(vfnSetUTCOffset());		// Determinate date and hour in seconds since 1980 and return value in 4 bytes

		stDataGPS -> DateInSeconds = ldwDateinSeconds;

		lflSpeed = flfnGetVel();
		stDataGPS -> Speed = lflSpeed;

		EnableIRQ(GPS_UART_IRQn);		//Enable UART 4 (GPS) interrupts
		return GPS_DATA_OK;				//GPS data ok

	}

	EnableIRQ(GPS_UART_IRQn);		//Enable UART 4 (GPS) interrupts
	return GPS_DATA_TIMEOUT;		//GPS starting
}


/********************* GPS UART 4 Handler *********************/

/*
 * fn:		void GPS_UART_IRQHandler(void)
 * brief:	ISR for UART channel 4, this function is going to save new gps data
 */
void GPS_UART_IRQHandler(void)
{
    uint8_t data;

    /* If new data arrived. */
    if ((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & UART_GetStatusFlags(GPS_UART))
    {

        data = UART_ReadByte(GPS_UART);		//Get new byte in data variable

        if(data == 'R' && rxIndex == 0)
        {
        	gpsMessageBuffer[rxIndex] = data;	//Save new data in gps message array
        	rxIndex++;											//Increase rxIndex
        }

        if(rxIndex > 0 && data != '\n')
        {
        	gpsMessageBuffer[rxIndex] = data;	//Save new data in gps message array
        	rxIndex++;
        }

        if(rxIndex > 0 && data == '\n')
        {
        	gpsMessageBuffer[rxIndex] = data;	//Save new data in gps message array
        	rxIndex = 0;		//Reset index position
        }

    }
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
