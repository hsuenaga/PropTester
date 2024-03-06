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
	sendSignature();
	return true;
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
	return true;
}
