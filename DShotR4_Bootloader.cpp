#include "DShotR4.h"

bool
DShotR4::bl_enter()
{
	// disable GPIO/GTIO while setup.
	if (gpt_pwmChannelA_enable)
	{
		pinPeripheral(gpt_pwmPinA, pin_cfg_output_high);
	}
	if (gpt_pwmChannelB_enable)
	{
		pinPeripheral(gpt_pwmPinB, pin_cfg_output_high);
	}

	suspend();
	delay(1000); // wait for signal detection timeout.
	serialCore.begin(CHANNEL_B, gpt_pwmPinB, 19200);

	return true;
}

bool
DShotR4::bl_exit()
{
	// XXX: send exit command(0) to BLHeli.
	serialCore.end();

	// restore waveform transmission mode.
	dtc_dshot_init();
	gpt_dshot_init();

	return true;
}

bool
DShotR4::bl_open()
{
  serialCore.write(blheli_signature, sizeof(blheli_signature));

  return true;
}

int
DShotR4::bl_read()
{
  return serialCore.read();
}

size_t
DShotR4::bl_write(uint8_t data)
{
  return serialCore.write(data);
}

void
DShotR4::bl_flush()
{
  serialCore.flush();
}

int
DShotR4::bl_available()
{
  return serialCore.available();
}


