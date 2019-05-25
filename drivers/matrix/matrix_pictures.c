/*
 * matrix_pictures.c
 * Here are the pictures for the matrix
 *
 *  Created on: May 25, 2019
 *      Author: geckobit
 */

#include "matrix.h"

const uint8_t MATRIX_DATA_OKAY[8] = {
		0b00000000,
		0b00000001,
		0b00000011,
		0b00000110,
		0b01101100,
		0b00111000,
		0b00010000,
		0b00000000
};

const uint8_t MATRIX_DATA_ERROR[8] = {
		0b01000010,
		0b00100100,
		0b00011000,
		0b00011000,
		0b00100100,
		0b01000010,
		0b00000000,
		0b00000000
};
const uint8_t MATRIX_DATA_LOCK_OPEN[8] = {
		0b00111000,
		0b01000000,
		0b10000000,
		0b11111111,
		0b10000001,
		0b10000001,
		0b10000001,
		0b11111111
};
const uint8_t MATRIX_DATA_LOCK_CLOSED[8] = {
		0b00111100,
		0b01000010,
		0b10000001,
		0b11111111,
		0b10000001,
		0b10000001,
		0b10000001,
		0b11111111
};
const uint8_t MATRIX_DATA_FULL_PATTERN[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const uint8_t MATRIX_DATA_CLEAR[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

