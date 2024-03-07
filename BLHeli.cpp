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
BLHeli::recv(uint8_t *buf, size_t len)
{
	if (len > sizeof(txrxBuf) - 3)
	{
		return NONE;
	}

	int recvLen = (len == 0) ? 1 : len + 3;
	int n = port.readBytes(&txrxBuf[0], recvLen);
	if (n != recvLen) {
		counter.rx_timeout++;
		return NONE;
	}

	uint8_t code = NONE;
	if (len > 0)
	{
		uint16_t crc = crcCompute(&txrxBuf[0], len);
		if (txrxBuf[len] != (crc & 0xFF))
		{
			counter.rx_bad_crc++;
			return false;
		}
		if (txrxBuf[len + 1] != ((crc >> 8) & 0xFF))
		{
			counter.rx_bad_crc++;
			return false;
		}
		code = txrxBuf[len + 2];
	}
	else {
		code = txrxBuf[0];
	}

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

uint8_t
BLHeli::recvAck()
{
	return recv(NULL, 0);
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
	uint8_t buf[9];

	memset(&bootInfo, 0, sizeof(bootInfo));
	restart();
	restart();
	send(blheli_signature, sizeof(blheli_signature));

	size_t len = port.readBytes(buf, sizeof(buf));
	if (len != sizeof(buf)) {
		return false;
	}
	bootInfo.Revision = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
	bootInfo.Signature = (buf[4] << 8) | buf[5];
	bootInfo.Version = buf[6];
	bootInfo.Pages = buf[7];
	bootInfo.CommandStatus = buf[8];

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
	if (send(cmd, 2) == false)
	{
		return false;
	}

	return true;
}
