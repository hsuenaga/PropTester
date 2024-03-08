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

	struct __attribute__((__packed__)) escStatus_t {
		uint8_t unknown[3];
		uint32_t protocol;
		uint32_t good_frames;
		uint32_t bad_frames;
		uint32_t unknown2;
	};

	struct counter_t
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
	bool conected;
	bool address_present;
	bool buffer_present;
	uint8_t txrxBuf[256 + 3];
	bootInfo_t bootInfo;
	counter_t counter;
	escStatus_t status;

	Stream &port;

	uint16_t crcCompute(const uint8_t *buf, size_t len);
	bool send(const uint8_t *buf, size_t len);
	uint8_t recv(uint8_t *buf, size_t len, bool hasCRC = true);
	uint8_t recvAck();
	uint8_t observeResultCode(uint8_t code);
	void parseBootInfo(uint8_t (&buf)[8]);

public:
	BLHeli(Stream &);
	~BLHeli();

	bool begin();
	bool end();
	bool sendSignature();
	bool keepAlive();
	bool restart();

	uint8_t setAddress(uint16_t addr);
	uint8_t setBuffer(const uint8_t *buf, uint16_t len);
	uint8_t readDataRaw(uint8_t type, uint8_t *buf, uint16_t len);
	uint8_t writeDataRaw(uint8_t type);
	uint8_t verifyDataRaw(uint8_t type);
	uint8_t pageEraseRaw();

	bool readData(uint8_t type, uint16_t addr, uint8_t *buf, uint16_t len);
	bool writeData(uint8_t type, uint16_t addr, const uint8_t *buf, uint16_t len);
	bool verifyData(uint8_t type, uint16_t addr, const uint8_t *buf, uint16_t len);

	bool readFlash(uint16_t addr, uint8_t *buf, uint16_t len);
	bool writeFlash(uint16_t addr, const uint8_t *buf, uint16_t len);

	bool readEEPROM(uint16_t addr, uint8_t *buf, uint16_t len);
	bool writeEEPROM(uint16_t addr, const uint8_t *buf, uint16_t len);

	bool readSRAM(uint16_t addr, uint8_t *buf, uint16_t len);

	escStatus_t get_statusinfo();

	bootInfo_t get_bootinfo()
	{
		return bootInfo;
	};

	counter_t get_counter()
	{
		return counter;
	}
};

#endif
