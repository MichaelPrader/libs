/*************************************************************************
Title:    MRBus CRC Functions
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan Holmes <maverick@drgw.net>
File:     mrbus-crc.c
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2012 Nathan Holmes and Michael Petersen

    Original code developed by Nathan Holmes for PIC architecture.  Based
    on AVR port by Michael Prader.  Updates and compatibility fixes by
    Michael Petersen.
    
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License along
    with this program. If not, see http://www.gnu.org/licenses/
    
*************************************************************************/

/* CRC16 Lookup tables (High and Low Byte) for 4 bits per iteration. */
/* CRC16 implementation of X^16 + X^15 + X^2 + X^0, poly 0xA001, init value 0x0000 */
const uint8_t MRBus_CRC16_HighTable[16] =
{
	0x00, 0xA0, 0xE0, 0x40, 0x60, 0xC0, 0x80, 0x20,
	0xC0, 0x60, 0x20, 0x80, 0xA0, 0x00, 0x40, 0xE0
};
const uint8_t MRBus_CRC16_LowTable[16] =
{
	0x00, 0x01, 0x03, 0x02, 0x07, 0x06, 0x04, 0x05,
	0x0E, 0x0F, 0x0D, 0x0C, 0x09, 0x08, 0x0A, 0x0B
};


uint16_t mrbusCRC16Update(uint16_t crc, uint8_t a)
{
	uint8_t t;
	uint8_t i = 0;

	uint8_t W;
	uint8_t crc16_high = (crc >> 8) & 0xFF;
	uint8_t crc16_low = crc & 0xFF;

	while (i < 2)
	{
		if (i)
		{
			W = ((crc16_high << 4) & 0xF0) | ((crc16_high >> 4) & 0x0F);
			W = W ^ a;
			W = W & 0x0F;
			t = W;
		}
		else
		{
			W = crc16_high;
			W = W ^ a;
			W = W & 0xF0;
			t = W;
			t = ((t << 4) & 0xF0) | ((t >> 4) & 0x0F);
		}

		crc16_high = crc16_high << 4;
		crc16_high |= (crc16_low >> 4);
		crc16_low = crc16_low << 4;

		crc16_high = crc16_high ^ MRBus_CRC16_HighTable[t];
		crc16_low = crc16_low ^ MRBus_CRC16_LowTable[t];

		i++;
	}

	return ( ((crc16_high << 8) & 0xFF00) + crc16_low );
}





