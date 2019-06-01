/*
 * matrix.c
 *
 *  Created on: May 23, 2019
 *      Author: geckobit
 */

#include "matrix.h"

/* These come from extern in header file */
EventGroupHandle_t matrix_ev_group;
uint32_t * matrix_pictures[MATRIX_PIC_TOTAL];

dspi_master_config_t _spi_cfg;
dspi_transfer_t _spi_transfer;
uint8_t spi_transfer_data[SPI_TRANSFER_SIZE];
uint8_t * _picture_to_print;
uint8_t _error_code;

void matrix_spi_init() {
	_spi_cfg.whichCtar = kDSPI_Ctar1;
	_spi_cfg.ctarConfig.baudRate = SPI_BAUDRATE;
	_spi_cfg.ctarConfig.bitsPerFrame = 8U;
	_spi_cfg.ctarConfig.cpol = kDSPI_ClockPolarityActiveHigh;
	_spi_cfg.ctarConfig.cpha = kDSPI_ClockPhaseFirstEdge;
	_spi_cfg.ctarConfig.direction = kDSPI_MsbFirst;
	_spi_cfg.ctarConfig.pcsToSckDelayInNanoSec = 0200000000U / SPI_BAUDRATE;
	_spi_cfg.ctarConfig.lastSckToPcsDelayInNanoSec = 100000000U / SPI_BAUDRATE;
	_spi_cfg.ctarConfig.betweenTransferDelayInNanoSec = 1000000U / SPI_BAUDRATE;

	_spi_cfg.whichPcs = SPI_INIT_PCS;
	_spi_cfg.pcsActiveHighOrLow = kDSPI_PcsActiveLow;

	_spi_cfg.enableContinuousSCK = false;
	_spi_cfg.enableRxFifoOverWrite = false;
	_spi_cfg.enableModifiedTimingFormat = false;
	_spi_cfg.samplePoint = kDSPI_SckToSin0Clock;

	_spi_transfer.txData = spi_transfer_data;
	_spi_transfer.dataSize = SPI_TRANSFER_SIZE;
	_spi_transfer.configFlags = kDSPI_MasterCtar0 | SPI_TRANSFER_PCS;
	DSPI_MasterInit(SPI_ADDR, &_spi_cfg, SPI_CLK_FREQ);
}

void matrix_init() {
	matrix_ev_group = xEventGroupCreate();
	matrix_pictures[MATRIX_PIC_OK] = (uint32_t *) MATRIX_DATA_OKAY;
	matrix_pictures[MATRIX_PIC_ERROR] = (uint32_t *) MATRIX_DATA_ERROR;
	matrix_pictures[MATRIX_PIC_LOCK_OPEN] = (uint32_t *) MATRIX_DATA_LOCK_OPEN;
	matrix_pictures[MATRIX_PIC_LOCK_CLOSE] = (uint32_t *) MATRIX_DATA_LOCK_CLOSED;
	matrix_pictures[MATRIX_PIC_CLEAR] = (uint32_t *) MATRIX_DATA_CLEAR;
	matrix_pictures[MATRIX_PIC_FULL] = (uint32_t *) MATRIX_DATA_FULL_PATTERN;
	matrix_pictures[MATRIX_PIC_WAIT_CLOCK] = (uint32_t *) MATRIX_DATA_WAIT_CLOCK;

	_picture_to_print = (uint8_t *) matrix_pictures[MATRIX_PIC_LOCK_CLOSE];
	matrix_write(MATRIX_CMD_SHUTDOWN, 0x0);
	matrix_write(MATRIX_CMD_SHUTDOWN, 0x1);
	matrix_write(MATRIX_CMD_SCAN_LIMIT, 0x7);
	matrix_write(MATRIX_CMD_INTENSITY, 0xA);
	matrix_print();
}

void matrix_send(matrix_pic picture) {
	_picture_to_print = (uint8_t *) matrix_pictures[picture];
	xEventGroupSetBits(matrix_ev_group, MATRIX_EVENT_SEND);
}

void matrix_send_err(matrix_err err_code) {
	_picture_to_print = (uint8_t *) matrix_pictures[MATRIX_PIC_ERROR];
	_error_code = err_code;
	xEventGroupSetBits(matrix_ev_group, MATRIX_EVENT_SEND);
}

void matrix_print() {
	uint8_t i;
	for (i = 1; i < 9; i++) {
		matrix_write(i, _picture_to_print[i - 1]);
	}
	if (_error_code) {
		matrix_write(MATRIX_CMD_7, _error_code);
		_error_code = 0;
	}
}

void matrix_write(matrix_cmd cmd, uint8_t data) {
	spi_transfer_data[1] = cmd;
	spi_transfer_data[0] = data;
	DSPI_MasterTransferBlocking(SPI_ADDR, &_spi_transfer);
}

