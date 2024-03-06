#pragma once

#include <Arduino.h>

#ifndef __BLHELI_H__
#define __BLHELI_H__
class BLHeli
{
private:
	const uint8_t blheli_signature[21] = {
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x0D,
	    'B', 'L', 'H', 'e', 'l', 'i',
	    0xF4, 0x7D};
	uint8_t bootInfo[9];
	bool conected;

	Stream &port;

public:
	BLHeli(Stream &);
	~BLHeli();

	bool begin();
	bool end();
	bool sendSignature();
};

#endif
