/*
 * Copyright (C) 2015 Motorola Mobility, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>

#define POLYNOMIAL		0x8005
#define CRC_HIGHBIT		0x8000

uint16_t gen_crc16(uint8_t *data, unsigned long len)
{
	uint16_t i, j, c, bit;
	uint16_t crc = 0;

	for (i = 0; i < len; i++) {
		c = (uint16_t)*data++;
		for (j = 0x80; j; j >>= 1) {
			bit = crc & CRC_HIGHBIT;
			crc <<= 1;
			if (c & j)
				bit ^= CRC_HIGHBIT;
			if (bit)
				crc ^= POLYNOMIAL;
		}
	}

	crc = (crc >> 8) | (crc << 8);
	return crc;
}
