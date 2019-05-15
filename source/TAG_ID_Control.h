/*
 * TAG_ID_Control.h
 *
 *  Created on: 5 mar. 2019
 *      Author: Dell
 */

#ifndef TAG_ID_CONTROL_H_
#define TAG_ID_CONTROL_H_




void vfcnSaveID (unsigned char * ID);
void vfcnValidateID (void);
void vfcndeleteID (void);


void error_trap(void);

void flash_init(void);
void flash_read (void);
void flash_write (void);
void flash_delete (void);

void GPIOinit(void);
void vfcnToggleLed (uint8_t Color);
void delay(void);

#endif /* TAG_ID_CONTROL_H_ */
