#include "BLHeli.h"

uint16_t
BLHeli::crcCompute(const uint8_t *buf, size_t len)
{
	const uint16_t poly = 0xA001;
	uint16_t crc = 0;

	while (len--) {
		uint8_t xb = *buf++;

		for (int i = 0; i < 8; i++) {
			if (((xb & 0x01) ^ (crc & 0x0001)) != 0) {
				crc >>= 1;
				crc = crc ^ poly;
			}
			else {
				crc >>= 1;
			}
			xb >>= 1;
		}
	}

	return crc;
}

bool
BLHeli::send(const uint8_t *buf, size_t len)
{
	if (len > sizeof(txrxBuf))
	{
		counter.tx_buffer_exhausted++;
		return false;
	}
	memcpy(txrxBuf, buf, len);
	uint16_t crc = crcCompute(buf, len);
	txrxBuf[len] = crc & 0xFF;
	txrxBuf[len + 1] = (crc >> 8) & 0xFF;

	size_t n = port.write(&txrxBuf[0], len + 2);
	if (n != (len + 2)) {
		counter.tx_failure++;
		return false;
	}
	counter.tx_success++;
	return true;
}

uint8_t
BLHeli::recv(uint8_t *buf, size_t len, bool hasCRC)
{
	int recvLen = hasCRC ? len + 3 : len + 1;
	if (recvLen > sizeof(txrxBuf)) {
		counter.rx_buffer_exhausted++;
		return NONE;
	}

	int n = port.readBytes(&txrxBuf[0], recvLen);
	if (n != recvLen)
	{
		counter.rx_timeout++;
		return NONE;
	}

	if (hasCRC)
	{
		uint16_t crc = crcCompute(&txrxBuf[0], len);
		if (txrxBuf[len] != (crc & 0xFF))
		{
			counter.rx_bad_crc++;
			return NONE;
		}
		if (txrxBuf[len + 1] != ((crc >> 8) & 0xFF))
		{
			counter.rx_bad_crc++;
			return NONE;
		}
	}

	uint8_t code = observeResultCode(txrxBuf[recvLen - 1]);
	if (code == SUCCESS && buf != NULL && len > 0) {
		memcpy(buf, &txrxBuf[0], len);
	}
	return code;
}

uint8_t
BLHeli::recvAck()
{
	return recv(NULL, 0, false);
}

uint8_t
BLHeli::observeResultCode(uint8_t code)
{
	switch (code)
	{
	case SUCCESS:
		counter.rx_success++;
		break;
	case ERRORVERIFY:
		counter.rx_error_verify++;
		break;
	case ERRORCOMMAND:
		counter.rx_error_command++;
		break;
	case ERRROCRC:
		counter.rx_error_crc++;
		break;
	case NONE:
	default:
		counter.rx_error_unknown++;
		break;
	}

	return code;
}

void
BLHeli::parseBootInfo(uint8_t (&buf)[8])
{
	bootInfo.Revision = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
	bootInfo.Signature = (buf[4] << 8) | buf[5];
	bootInfo.Version = buf[6];
	bootInfo.Pages = buf[7];

	switch (bootInfo.Signature)
	{
	case 0x9307:
	case 0x930A:
	case 0x930F:
	case 0x940B:
		bootInfo.CPUType = ATMEL;
		break;
	case 0xF310:
	case 0xF330:
	case 0xF410:
	case 0xF390:
	case 0xF850:
	case 0xE8B1:
	case 0xE8B2:
		bootInfo.CPUType = SILAB;
		break;
	default:
		if (buf[4] == 0x06 && buf[5] > 0x00 && buf[5] < 0x90)
		{
			bootInfo.CPUType = ARM;
		}
		else
		{
			bootInfo.CPUType = UNKNOWN;
		}
		break;
	}
}

BLHeli::BLHeli(Stream &s) : port(s)
{
}

BLHeli::~BLHeli()
{
}

bool
BLHeli::begin()
{
	return sendSignature();
}

bool
BLHeli::end()
{
	// XXX: send exit command(0) to BLHeli.

	return true;
}

bool
BLHeli::sendSignature()
{
	uint8_t buf[8];

	memset(&bootInfo, 0, sizeof(bootInfo));
	restart();
	restart();
	send(blheli_signature, sizeof(blheli_signature));

	uint8_t code = recv(buf, sizeof(buf), false);
	if (code != SUCCESS) {
		return false;
	}

	parseBootInfo(buf);
	return true;
}

bool
BLHeli::keepAlive()
{
	// Actually, command KEEP_ALIVE is not implemented on ESCs.
	// ERRORCOMMAND will be returned if ESC works fine.
	uint8_t cmd[] = {KEEP_ALIVE, 0};
	if (send(cmd, 2) == false)
	{
		return false;
	}

	uint8_t code = recvAck();
	if (code == ERRORCOMMAND)
	{
		return true;
	}

	return false;
}

bool
BLHeli::restart()
{
	uint8_t cmd[] = {RUN, 0};
	if (send(cmd, sizeof(cmd)) == false)
	{
		return false;
	}

	return true;
}

bool
BLHeli::setAddress(uint16_t addr)
{
	if (addr == 0xFFFF) {
		return true;
	}

	uint8_t cmd[4] = {
	    SET_ADDRESS, 0,
	    ((addr >> 8) & 0xFF),
	    (addr & 0xFF)
	};

	if (send(cmd, sizeof(cmd)) == false)
	{
		return false;
	}
	return (recvAck() == SUCCESS);
}

bool
BLHeli::setBuffer(const uint8_t *buf, uint16_t len)
{
	uint8_t cmd[4] = {
		SET_BUFFER,
		0,
		(uint8_t)((len >> 8) & 0xFF),
		(uint8_t)(len & 0xFF)
	};

	if (send(cmd, sizeof(cmd)) == false)
	{
		return false;
	}
	if (recvAck() != SUCCESS) {
		return false;
	}

	if (send(buf, len) == false)
	{
		return false;
	}
	return (recvAck() == SUCCESS);
}

bool
BLHeli::readData(uint8_t type, uint8_t *buf, uint16_t len)
{
	if (len > 256) {
		return false;
	}

	uint8_t cmd[2] = {type, (uint8_t)(len == 256 ? 0 : len)};
	if (send(cmd, sizeof(cmd)) == false) {
		return false;
	}
	return (recv(buf, len) == SUCCESS);
}

bool
BLHeli::pageErase()
{
	uint8_t cmd[2] = {ERASE_FLASH, 0x01};

	if (send(cmd, sizeof(cmd)) == false) {
		return false;
	}

	port.setTimeout(3000);
	uint8_t code = recvAck();
	port.setTimeout(1000);

	return (code == SUCCESS);
}
