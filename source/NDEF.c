/*
 * NDEF.c
 *
 *  Created on: 28 may. 2019
 *      Author: Dell
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "pin_mux.h"
#include "clock_config.h"

#include "stdio.h"
#include "math.h"
#include "NDEF.h"


int integer_value;


char buffer_Longitud[10];
char buffer_Latitud[10];
char buffer_Distance[10];
char buffer_DateInSeconds[10];
char buffer_Speed[10];

char buffer_Year[10];
char buffer_Day[10];
char buffer_Month[10];
char buffer_Hour[10];
char buffer_Minute[10];
char buffer_Second[10];
char buffer_Date[12];
char buffer_time[10];

float Float_Longitud;
float Float_Latitud;
float Float_Distance;
unsigned int Int_DateInSeconds;
float Float_Speed;
float meanSpeed;
unsigned int Int_DateInSeconds1;
unsigned int Int_DateInSeconds2;
unsigned int Int_Total_Time;





//Calculate Date and transfor to Character
void EPOCH_Converter (unsigned int epoch)
{
	static unsigned char month_days[12]={31,28,31,30,31,30,31,31,30,31,30,31};
	static unsigned char week_days[7] = {4,5,6,0,1,2,3}; /* Thu=4, Fri=5, Sat=6, Sun=0, Mon=1, Tue=2, Wed=3 */
	unsigned char ntp_week_day, leap_days, leap_year_ind ;
	unsigned short temp_days;
	unsigned int days_since_epoch, day_of_year;
	unsigned int ntp_year;
	unsigned char ntp_hour, ntp_minute, ntp_second, ntp_date, ntp_month;

    leap_days=0;
    leap_year_ind=0;
    /* Add or substract time zone here.
     epoch+=19800 ; //GMT +5:30 = +19800 seconds */
    epoch-=0x000 ;


    ntp_second = epoch%60;
    epoch /= 60;
	ntp_minute = epoch%60;
	epoch /= 60;
	ntp_hour  = epoch%24;
	epoch /= 24;

	days_since_epoch = epoch;      //number of days since epoch
	ntp_week_day = week_days[days_since_epoch%7];  //Calculating WeekDay

    ntp_year = 1970+(days_since_epoch/365); // ball parking year, may not be accurate!

    int i;
    for (i=1972; i<ntp_year; i+=4)      // Calculating number of leap days since epoch/1970
    if(((i%4==0) && (i%100!=0)) || (i%400==0)) leap_days++;

    ntp_year = 1980+((days_since_epoch - leap_days)/365); // Calculating accurate current year by (days_since_epoch - extra leap days)
    day_of_year = ((days_since_epoch - leap_days)%365)+1;


	if(((ntp_year%4==0) && (ntp_year%100!=0)) || (ntp_year%400==0))
	{
		 month_days[1]=29;     //February = 29 days for leap years
		 leap_year_ind = 1;    //if current year is leap, set indicator to 1
	}
	else month_days[1]=28; //February = 28 days for non-leap years

	temp_days=0;

	for (ntp_month=0 ; ntp_month <= 11 ; ntp_month++) //calculating current Month
	{
		   if (day_of_year <= temp_days) break;
		   temp_days = temp_days + month_days[ntp_month];
	}

    temp_days = temp_days - month_days[ntp_month-1]; //calculating current Date
    ntp_date = day_of_year - temp_days;


	  // -------------------- Printing day -------------------------------------
      //printf(" %d -",ntp_date);
      integer_value = ntp_date;
      sprintf(buffer_Day, "%d", integer_value);
      // -------------------- Printing Month -------------------------------------
     // printf(" %d -",ntp_month);
      integer_value = ntp_month;
      sprintf(buffer_Month, "%d", integer_value);

      // -------------------- Printing Year -------------------------------------
     // printf(" %d\n",ntp_year);
      sprintf(buffer_Year, "%d", ntp_year);

      // -------------------- Printing Time -------------------------------------
      //printf("TIME = %2d : %2d : %2d\n\n", ntp_hour,ntp_minute,ntp_second)  ;
      integer_value = ntp_hour;
      sprintf(buffer_Hour, "%d", integer_value);
      integer_value = ntp_minute;
      sprintf(buffer_Minute, "%d", integer_value);
      integer_value = ntp_second;
      sprintf(buffer_Second, "%d", integer_value);



}

// reverses a string 'str' of length 'len'
void reverse(char *str, int len)
{
    int i=0, j=len-1, temp;
    while (i<j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++; j--;
    }
}

int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x)
    {
        str[i++] = (x%10) + '0';
        x = x/10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

// Converts a floating point number to string.
void Float_To_String(float n, char *res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // convert integer part to string
    int i = intToStr(ipart, res, 0);

    // check for display option after point
    if (afterpoint != 0)
    {
        res[i] = '.';  // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);

        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}



void Convert_To_NDEF_Message (stGPSData_t GPS, unsigned int flag, char * ndefarr )
{

	Float_Latitud = GPS.Latitude;
	Float_Longitud = GPS.Longitude;
	Float_Distance = GPS.Distance;
	Int_DateInSeconds = GPS.DateInSeconds;
	Float_Speed = GPS.Speed;
	int lb_integer_value;

	if(flag == 1)
	{

		Int_DateInSeconds1 = Int_DateInSeconds;
	    EPOCH_Converter(Int_DateInSeconds);
	    /* Day */
	    if(buffer_Day[1] == 0)
	    {
	    ndefarr[0] = '0';
	    ndefarr[1] = buffer_Day[0];
	    }
	    else{
	    ndefarr[0] = buffer_Day[0];
	    ndefarr[1] = buffer_Day[1];}

	    /* Month */
	    if(buffer_Month[1] == 0)
	    {
	    ndefarr[2] = '0';
	    ndefarr[3] = buffer_Month[0];
	    }
	    else{
	    ndefarr[2] = buffer_Month[0];
	    ndefarr[3] = buffer_Month[1];}

	    /* Year */
	    ndefarr[4] = buffer_Year[0];
	    ndefarr[5] = buffer_Year[1];
	    ndefarr[6] = buffer_Year[2];
	    ndefarr[7] = buffer_Year[3];

	    /* Hour */
	    if(buffer_Hour[1] == 0)
	    {
	    ndefarr[8] = '0';
	    ndefarr[9] = buffer_Hour[0];
	    }
	    else{
	    ndefarr[8] = buffer_Hour[0];
	    ndefarr[9] = buffer_Hour[1];}

	    /* Minute */
	    if(buffer_Minute[1] == 0)
	    {
	    ndefarr[10] = '0';
	    ndefarr[11] = buffer_Minute[0];
	    }
	    else{
	    ndefarr[10] = buffer_Minute[0];
	    ndefarr[11] = buffer_Minute[1];}

	    /* Latitud */
	    if (Float_Latitud > 0)
	    {Float_To_String(Float_Latitud, buffer_Latitud, 6);
	    ndefarr[12] = '+';}
	    else if (Float_Latitud < 0){
	    Float_Latitud = fabs(Float_Latitud);
	    ndefarr[12] = '-';
		Float_To_String(Float_Latitud, buffer_Latitud, 6);}
	    ndefarr[13] = buffer_Latitud[0];
	    ndefarr[14] = buffer_Latitud[1];
	    ndefarr[15] = buffer_Latitud[2];
	    ndefarr[16] = buffer_Latitud[3];
	    ndefarr[17] = buffer_Latitud[4];
	    ndefarr[18] = buffer_Latitud[5];
	    ndefarr[19] = buffer_Latitud[6];
	    ndefarr[20] = buffer_Latitud[7];

	    /* Longitud */
	    if (Float_Longitud > 0)
	    {Float_To_String(Float_Longitud, buffer_Longitud, 6);
	    ndefarr[21] = '+';}
	    else if (Float_Longitud < 0){
		Float_Longitud = fabs(Float_Longitud);
		ndefarr[21] = '-';
		Float_To_String(Float_Longitud, buffer_Longitud, 6);}
		ndefarr[22] = buffer_Longitud[0];
		ndefarr[23] = buffer_Longitud[1];
		ndefarr[24] = buffer_Longitud[2];
		ndefarr[25] = buffer_Longitud[3];
		ndefarr[26] = buffer_Longitud[4];
		ndefarr[27] = buffer_Longitud[5];
		ndefarr[28] = buffer_Longitud[6];
		ndefarr[29] = buffer_Longitud[7];
	}

	if(flag == 2)
	{
		lflCurrentDistance = 0;
		Int_DateInSeconds2 = Int_DateInSeconds;
		Int_Total_Time = (Int_DateInSeconds2 - Int_DateInSeconds1);
		if(Float_Distance < 1)
			Float_Distance = 0;
		if(Int_Total_Time == 0)
		{
			meanSpeed = 0;
		}
		else if(Int_Total_Time >= 1u)
		{
			meanSpeed = (Float_Distance*36)/((float)Int_Total_Time*10);
		}
		Int_Total_Time = Int_Total_Time / 60;
		lb_integer_value = Int_Total_Time;
		sprintf(buffer_time, "%d", lb_integer_value);

	    EPOCH_Converter(Int_DateInSeconds);
	    /* Day */
	    if(buffer_Day[1] == 0)
	    {
	    ndefarr[30] = '0';
	    ndefarr[31] = buffer_Day[0];
	    }
	    else{
	    ndefarr[30] = buffer_Day[0];
	    ndefarr[31] = buffer_Day[1];}

	    /* Month */
	    if(buffer_Month[1] == 0)
	    {
	    ndefarr[32] = '0';
	    ndefarr[33] = buffer_Month[0];
	    }
	    else{
	    ndefarr[32] = buffer_Month[0];
	    ndefarr[33] = buffer_Month[1];}

	    /* Year */
	    ndefarr[34] = buffer_Year[0];
	    ndefarr[35] = buffer_Year[1];
	    ndefarr[36] = buffer_Year[2];
	    ndefarr[37] = buffer_Year[3];

	    /* Hour */
	    if(buffer_Hour[1] == 0)
	    {
	    ndefarr[38] = '0';
	    ndefarr[39] = buffer_Hour[0];
	    }
	    else{
	    ndefarr[38] = buffer_Hour[0];
	    ndefarr[39] = buffer_Hour[1];}

	    /* Minute */
	    if(buffer_Minute[1] == 0)
	    {
	    ndefarr[40] = '0';
	    ndefarr[41] = buffer_Minute[0];
	    }
	    else{
	    ndefarr[40] = buffer_Minute[0];
	    ndefarr[41] = buffer_Minute[1];}

	    /* Latitud */
	    if (Float_Latitud > 0)
	    {Float_To_String(Float_Latitud, buffer_Latitud, 6);
	    ndefarr[42] = '+';}
	    else if (Float_Latitud < 0){
	    Float_Latitud = fabs(Float_Latitud);
	    ndefarr[42] = '-';
		Float_To_String(Float_Latitud, buffer_Latitud, 6);}
	    ndefarr[43] = buffer_Latitud[0];
	    ndefarr[44] = buffer_Latitud[1];
	    ndefarr[45] = buffer_Latitud[2];
	    ndefarr[46] = buffer_Latitud[3];
	    ndefarr[47] = buffer_Latitud[4];
	    ndefarr[48] = buffer_Latitud[5];
	    ndefarr[49] = buffer_Latitud[6];
	    ndefarr[50] = buffer_Latitud[7];

	    /* Longitud */
	    if (Float_Longitud > 0)
	    {Float_To_String(Float_Longitud, buffer_Longitud, 6);
	    ndefarr[51] = '+';}
	    else if (Float_Longitud < 0){
		Float_Longitud = fabs(Float_Longitud);
		ndefarr[51] = '-';
		Float_To_String(Float_Longitud, buffer_Longitud, 6);}
		ndefarr[52] = buffer_Longitud[0];
		ndefarr[53] = buffer_Longitud[1];
		ndefarr[54] = buffer_Longitud[2];
		ndefarr[55] = buffer_Longitud[3];
		ndefarr[56] = buffer_Longitud[4];
		ndefarr[57] = buffer_Longitud[5];
		ndefarr[58] = buffer_Longitud[6];
		ndefarr[59] = buffer_Longitud[7];

		/* JOURNEY MINUTES */
		ndefarr[60] = buffer_time[0];
		ndefarr[61] = buffer_time[1];
		ndefarr[62] = buffer_time[2];


		/* SPEED */
		Float_To_String(meanSpeed, buffer_Speed, 6);
		ndefarr[63] = buffer_Speed[0];
		ndefarr[64] = buffer_Speed[1];
		ndefarr[65] = buffer_Speed[2];
		ndefarr[66] = buffer_Speed[3];
		ndefarr[67] = buffer_Speed[4];
		ndefarr[68] = buffer_Speed[5];

		/* DISTANCE */
		Float_To_String(Float_Distance, buffer_Distance, 6);
		ndefarr[69] = buffer_Distance[0];
		ndefarr[70] = buffer_Distance[1];
		ndefarr[71] = buffer_Distance[2];
		ndefarr[72] = buffer_Distance[3];
		ndefarr[73] = buffer_Distance[4];
		ndefarr[74] = buffer_Distance[5];
		ndefarr[75] = buffer_Distance[6];
		ndefarr[76] = buffer_Distance[7];

	}
}










