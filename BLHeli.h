#pragma once

#include <Arduino.h>

#ifndef __BLHELI_H__
#define __BLHELI_H__
class BLHeli
{
public:
	enum cpuType_t
	{
		UNKNOWN = 0,
		ARM,
		SILAB,
		ATMEL,
	};
	struct bootInfo_t
	{
		uint32_t Revision;
		uint16_t Signature;
		uint8_t Version;
		uint8_t Pages;

		cpuType_t CPUType;
	};

	enum resultCode_t
	{
		SUCCESS = 0x30,
		ERRORVERIFY = 0xC0,
		ERRORCOMMAND = 0xC1,
		ERRROCRC = 0xC2,
		NONE = 0xFF
	};

	enum commandCode_t
	{
		RUN = 0x00,
		PROG_FLASH = 0x01,
		ERASE_FLASH = 0x02,
		READ_FLASH_SIL = 0x03,
		VERIFY_FLASH = 0x03,
		VERIFY_FLASH_ARM = 0x04,
		READ_EEPROM = 0x04,
		PROG_EEPROM = 0x05,
		READ_SRAM = 0x06,
		READ_FLASH_ATM = 0x07,

		KEEP_ALIVE = 0xFD,
		SET_BUFFER = 0xFE,
		SET_ADDRESS = 0xFF,
	};

	enum escVariable_t
	{
		VAR_STATUS = 0xEB00,
	};

	enum escProtocl_t
	{
		PROTO_NONE = 0,
		PROTO_NORMAL = 1,
		PROTO_ONESHOT125 = 2,
		PROTO_DSHOT = 5,
	};

	struct __attribute__((__packed__)) EscStatus_t {
		uint8_t unknown[3];
		uint32_t protocol;
		uint32_t good_frames;
		uint32_t bad_frames;
		uint32_t unknown2;
	};

	struct Counter_t
	{
		uint32_t tx_buffer_exhausted;
		uint32_t tx_failure;
		uint32_t tx_success;

		uint32_t rx_buffer_exhausted;
		uint32_t rx_timeout;
		uint32_t rx_bad_crc;
		uint32_t rx_success;
		uint32_t rx_error_verify;
		uint32_t rx_error_command;
		uint32_t rx_error_crc;
		uint32_t rx_error_unknown;
	};

private:
	const uint8_t blheli_signature[6] = {'B', 'L', 'H', 'e', 'l', 'i'};
	bootInfo_t bootInfo;
	bool conected;
	uint8_t txrxBuf[256 + 3];
	Counter_t counter;

	Stream &port;

	uint16_t crcCompute(const uint8_t *buf, size_t len);
	bool send(const uint8_t *buf, size_t len);
	uint8_t recv(uint8_t *buf, size_t len, bool hasCRC = true);
	uint8_t recvAck();
	uint8_t observeResultCode(uint8_t code);

public:
	BLHeli(Stream &);
	~BLHeli();

	bool begin();
	bool end();
	bool sendSignature();
	bool keepAlive();
	bool restart();
	bool setAddress(uint16_t addr);
	bool readData(uint8_t type, uint8_t *buf, uint16_t len);
	bool pageErase();

	bootInfo_t get_bootinfo()
	{
		return bootInfo;
	};

	Counter_t get_counter()
	{
		return counter;
	}
};

#endif
