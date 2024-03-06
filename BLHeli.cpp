#include "BLHeli.h"

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
	port.write(blheli_signature, sizeof(blheli_signature));

	size_t len = port.readBytes(bootInfo, sizeof(bootInfo));
	if (len != sizeof(bootInfo)) {
		return false;
	}

	return true;
}
