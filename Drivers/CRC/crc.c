/*
 * crc.c
 *
 *  Created on: Oct 16, 2023
 *      Author: baris
 */
#include <crc.h>


uint32_t crc32_byte(uint8_t *p, uint32_t bytelength)
{
	uint32_t crc = 0xffffffff;
	while (bytelength-- !=0) crc = poly8_lookup[((uint8_t) crc ^ *(p++))] ^ (crc >> 8);
	// return (~crc); also works
	return (crc ^ 0xffffffff);
}
