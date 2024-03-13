#pragma once

#include <Arduino.h>
#include <FspTimer.h>

#ifndef __SCS0009_H__
#define __SCS0009_H__
class SCS0009
{
public:
	enum baud_t
	{
		B1000000 = 0x00,
		B500000 = 0x01,
		B250000 = 0x02,
		B128000 = 0x03,
		B115200 = 0x04,
		B76800 = 0x05,
		B57600 = 0x06,
		B38400 = 0x07,
	};

private:
	Stream &port;

	enum command_t
	{
		READ = 0x02,
		WRITE = 0x03,
	};

	enum result_t
	{
		SUCCESS = 0x00,
	};

	enum address_t
	{
		MAJOR_VERSION = 0x00,
		MINOR_VERSION = 0x01,
		MASTER_VERSION = 0x03,
		SUB_VERSION = 0x04,
		ID = 0x05,
		BAUD = 0x06,
		DELAY = 0x07,
		RSPONSE_LEVEL = 0x08,

		MIN_ANGLE = 0x09, // 2 bytes [step]
		MAX_ANGLE = 0x0B, // 2 bytes [step]

		MAX_TEMP = 0x0D, // 1 bytes [degC]
		MAX_VOLTAGE = 0x0E, // 1 bytes [0.1V]
		MIN_VOLTAGE = 0x0F, // 1 bytes [0.1V]

		TARGET_ANGLE = 0x2A, // 2 bytes [steps]

    		CURRENT_POSITION = 0x38, // 2 bytes [step]
		CURRENT_VELOCITY = 0x3A, // 2 bytes [step/s]
		CURRENT_LOAD = 0x3C, // 2 bytes [0.1%]
		CURRENT_VOLTAGE = 0x3E, // 1 byte [0.1V]
		CURRENT_TEMP = 0x3F, // 1 byte [degC]
	};

	static const int MaxDataLen = 16;
	struct __attribute__((__packed__)) tx_header_t
	{
		uint8_t dummy; // 0xff
		uint8_t stx; // 0xff

		uint8_t id;
		uint8_t len;
		uint8_t command;
		uint8_t address;
		// followed by data and Checksum
	};

	struct __attribute__((__packed__)) rx_header_t
	{
		uint8_t dummy; // 0xff
		uint8_t stx; // 0xff

		uint8_t id;
		uint8_t len;
		uint8_t result;
		// followed by data and Checksum
	};

	using frame_buffer_t = uint8_t[16]; // tx/rx FIFO size.

 	uint8_t calcChecksum(frame_buffer_t &frame, off_t off, size_t len);
	uint8_t addChecksum(frame_buffer_t &frame, size_t len);
	bool compChecksum(frame_buffer_t &frame);

	bool readRegister(uint8_t id, address_t addr, uint8_t *buf, uint8_t len);
	int read8(uint8_t id, address_t addr);
	int read16(uint8_t id, address_t addr);

	bool writeRegister(uint8_t id, address_t addr, const uint8_t *buf, uint8_t len);
	ssize_t write8(uint8_t id, address_t addr, uint8_t val);
	ssize_t write16(uint8_t id, address_t addr, uint16_t val);

public:
	SCS0009(Stream &p);
	~SCS0009();

	void begin();
	void end();

	int readVersion(uint8_t id);
	int readServoVersion(uint8_t id);
	int readBaud(uint8_t id);
	int writeBaud(uint8_t id, baud_t baud);

	int getPosition(uint8_t id);
};
#endif /* __SCS0009_H__ */
