/*
 * matrix.h
 *
 *  Created on: May 23, 2019
 *      Author: geckobit
 */

#ifndef MATRIX_MATRIX_H_
#define MATRIX_MATRIX_H_

#include "FreeRTOS.h"
#include "event_groups.h"
#include "fsl_device_registers.h"
#include "fsl_dspi.h"
#include "board.h"

#define SPI_ADDR SPI0
#define SPI_CLK_SRC DSPI0_CLK_SRC
#define SPI_CLK_FREQ CLOCK_GetFreq(DSPI0_CLK_SRC)
#define SPI_INIT_PCS kDSPI_Pcs0
#define SPI_TRANSFER_PCS kDSPI_MasterPcs0
#define SPI_BAUDRATE 4000000U
#define SPI_TRANSFER_SIZE 2U
#define MATRIX_POWER_PIN 4
#define MATRIX_EVENT_SEND (1 << 0)

typedef enum {
	MATRIX_PIC_OK,
	MATRIX_PIC_ERROR,
	MATRIX_PIC_LOCK_OPEN,
	MATRIX_PIC_LOCK_CLOSE,
	MATRIX_PIC_CLEAR,
	MATRIX_PIC_FULL,
	MATRIX_PIC_WAIT_CLOCK,
	MATRIX_PIC_TOTAL
} matrix_pic;

typedef enum {
	MATRIX_CMD_NO_OP,
	MATRIX_CMD_0,
	MATRIX_CMD_1,
	MATRIX_CMD_2,
	MATRIX_CMD_3,
	MATRIX_CMD_4,
	MATRIX_CMD_5,
	MATRIX_CMD_6,
	MATRIX_CMD_7,
	MATRIX_CMD_DECODE_MODE,
	MATRIX_CMD_INTENSITY,
	MATRIX_CMD_SCAN_LIMIT,
	MATRIX_CMD_SHUTDOWN,
	MATRIX_CMD_UNUSED1,
	MATRIX_CMD_UNUSED2,
	MATRIX_CMD_TEST
} matrix_cmd;

typedef enum {
	MATRIX_ERR_NONE,
	MATRIX_ERR_NFC_PERIPH_CONN,
	MATRIX_ERR_NFC_CFG,
	MATRIX_ERR_NFC_DRV,
	MATRIX_ERR_NFC_DISCOVERY
} matrix_err;

extern uint32_t * matrix_pictures[MATRIX_PIC_TOTAL];
extern EventGroupHandle_t matrix_ev_group;

/* Pictures */
extern const uint8_t MATRIX_DATA_OKAY[8];
extern const uint8_t MATRIX_DATA_ERROR[8];
extern const uint8_t MATRIX_DATA_LOCK_OPEN[8];
extern const uint8_t MATRIX_DATA_LOCK_CLOSED[8];
extern const uint8_t MATRIX_DATA_CLEAR[8];
extern const uint8_t MATRIX_DATA_FULL_PATTERN[8];
extern const uint8_t MATRIX_DATA_WAIT_CLOCK[8];

void matrix_spi_init(void);
void matrix_init(void);
void matrix_send(matrix_pic picture);
void matrix_send_err(matrix_err err_code);
void matrix_print();
void matrix_write(matrix_cmd cmd, uint8_t data);

#endif /* MATRIX_MATRIX_H_ */
