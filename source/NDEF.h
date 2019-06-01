/*
 * NDEF.h
 *
 *  Created on: 28 may. 2019
 *      Author: Dell
 */

#ifndef NDEF_H_
#define NDEF_H_


#include "gps.h"
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

void Float_To_String(float n, char *res, int afterpoint);
void Convert_To_NDEF_Message (stGPSData_t GPS, unsigned int flag, char * ndefarr );
void EPOCH_Converter (unsigned int epoch);




#endif /* NDEF_H_ */
