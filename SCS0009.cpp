#include "SCS0009.h"

SCS0009::SCS0009(Stream &p) : port(p)
{

}

SCS0009::~SCS0009()
{

}

void
SCS0009::begin()
{
}

void
SCS0009::end()
{
}

uint8_t
SCS0009::calcChecksum(frame_buffer_t &frame, off_t off, size_t len)
{
	uint8_t *datap = &frame[0];
	uint8_t sum = 0;

	for (int i = 0; i < len; i++)
	{
		sum += datap[off + i];
	}

	return ~sum;
}

uint8_t
SCS0009::addChecksum(frame_buffer_t &frame, size_t len)
{
	tx_header_t *header = (tx_header_t *)&frame[0];
	off_t off = offsetof(tx_header_t, id);
	size_t sumlen = sizeof(tx_header_t) - off + len;
	uint8_t *datap = (uint8_t *)(header + 1);
	uint8_t sum = calcChecksum(frame, off, sumlen);
	datap[len] = sum;

	return sum;
}

bool
SCS0009::compChecksum(frame_buffer_t &frame)
{
	rx_header_t *rhp = (rx_header_t *)&frame[0];
	uint8_t *dp = (uint8_t *)(rhp + 1);
	uint8_t len = rhp->len;

	off_t off = offsetof(rx_header_t, id);
	size_t sumlen = len + 1;

	uint8_t sum = calcChecksum(frame, off, sumlen);

	return (dp[len - 2] == sum);
}

bool
SCS0009::readRegister(uint8_t id, address_t addr, uint8_t *buf, uint8_t len)
{
	frame_buffer_t frame;
	tx_header_t *thp = (tx_header_t *)&frame[0];
	uint8_t *dp = (uint8_t *)(thp + 1);

	thp->dummy = 0xff;
	thp->stx = 0xff;
	thp->id = id;
	thp->len = 3 + 1;
	thp->command = READ;
	thp->address = addr;
	dp[0] = len;
	addChecksum(frame, 1);

	size_t frame_len = sizeof(tx_header_t) + 1 + 1;
	size_t n = port.write((uint8_t *)&frame, frame_len);
	if (n != frame_len)
	{
		return false;
	}

	size_t recv_len = sizeof(rx_header_t) + len + 1;
	n = port.readBytes((char *)&frame[0], recv_len);
	if (n != recv_len)
	{
		return false;
	}
	if (compChecksum(frame) == false)
	{
		return false;
	}
	rx_header_t *rhp = (rx_header_t *)&frame[0];
	dp = (uint8_t *)(rhp + 1);
	memcpy(buf, dp, len);

	return true;
}

int
SCS0009::read8(uint8_t id, address_t addr)
{
	uint8_t byte;

	if (readRegister(id, addr, &byte, sizeof(byte)) == false)
	{
		return -1;
	}

	return (int)(byte & 0xFF);
}

int
SCS0009::read16(uint8_t id, address_t addr)
{
	uint8_t bytes[2];

	if (readRegister(id, addr, &bytes[0], sizeof(bytes)) == false)
	{
		return -1;
	}

	return (int)(bytes[0] << 8 | bytes[1]);
}

bool
SCS0009::writeRegister(uint8_t id, address_t addr, const uint8_t *buf, uint8_t len)
{
	frame_buffer_t frame;
	tx_header_t *thp = (tx_header_t *)&frame[0];
	uint8_t *dp = (uint8_t *)(thp + 1);

	thp->dummy = 0xff;
	thp->stx = 0xff;
	thp->id = id;
	thp->len = 3 + len;
	thp->command = WRITE;
	thp->address = addr;
	memcpy(dp, buf, len);
	addChecksum(frame, len);

	size_t frame_len = sizeof(tx_header_t) + len + 1;
	size_t n = port.write((uint8_t *)&frame, frame_len);
	if (n != frame_len)
	{
		return false;
	}

	size_t recv_len = sizeof(rx_header_t) + 1;
	n = port.readBytes((char *)&frame[0], recv_len);
	if (n != recv_len)
	{
		return false;
	}
	if (compChecksum(frame) == false)
	{
		return false;
	}
	rx_header_t *rhp = (rx_header_t *)&frame[0];
	if (rhp->result != 0)
	{
		return false;
	}

	return true;
}

ssize_t
SCS0009::write8(uint8_t id, address_t addr, uint8_t val)
{
	if (writeRegister(id, addr, &val, sizeof(val)) == false)
	{
		return -1;
	}

	return sizeof(val);
}

ssize_t
SCS0009::write16(uint8_t id, address_t addr, uint16_t val)
{
	uint8_t buffer[2];

	buffer[0] = (val >> 8) & 0xFF;
	buffer[1] = val & 0xFF;

	if (writeRegister(id, addr, &buffer[0], sizeof(buffer)) == false)
	{
		return -1;
	}

	return sizeof(val);
}

int
SCS0009::readVersion(uint8_t id)
{
	return read16(id, MAJOR_VERSION);
}

int
SCS0009::readServoVersion(uint8_t id)
{
	return read16(id, MASTER_VERSION);
}

int
SCS0009::readBaud(uint8_t id)
{
	return read8(id, BAUD);
}

int
SCS0009::writeBaud(uint8_t id, baud_t baud)
{
	if (write8(id, BAUD, (uint8_t)(baud & 0xFF)) < 0)
	{
		return -1;
	}

	return (int)(baud);
}

int
SCS0009::getPosition(uint8_t id)
{
	return read16(id, CURRENT_POSITION);
}
