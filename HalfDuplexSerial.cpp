#include "HalfDuplexSerial.h"

#undef TX_DEBUG_LOW

void
HalfDuplexSerialCore::tx_serial_complete(timer_callback_args_t (*arg))
{
  tx_busy = false;
  tx_serial_success++;
  if (serialTxBytes > 0) {
    tx_serial_restart();
    return;
  }
  rx_serial_start();

  return;
}

void
HalfDuplexSerialCore::rx_serial_complete(timer_callback_args_t (*arg))
{
  rx_serial_detect++;
  rx_serial_restart(false);

  return;
}

void
HalfDuplexSerialCore::load_serial_frame(TimerPWMChannel_t ch)
{
  assert(tx_busy == false);
  assert(serialTxBits >= 12);

  uint8_t *p = serialTxLevel;

  *p++ = pin_cfg_input; // idle
  *p++ = pin_cfg_output_low; // start bit
  for (int i = 0; i < 8; i++) {
    bool bit = ((*serialTxPtr >> i) & 0x1) == 0x1;
    *p++ = (bit ? pin_cfg_output_high : pin_cfg_output_low) & 0xFF;
  }
  *p++ = pin_cfg_output_high; // stop bit
  *p++ = pin_cfg_input; // idle
}

void
HalfDuplexSerialCore::gpt_serial_init()
{
  float default_duty = 0.0;

  fsp_timer.set_frequency((float)bitRate); // this function starts timer automatically.
  fsp_timer.stop();
  fsp_timer.set_duty_cycle(default_duty, CHANNEL_A);
  fsp_timer.set_duty_cycle(default_duty, CHANNEL_B);
  fsp_timer.reset();
}

void
HalfDuplexSerialCore::dtc_serial_init(void)
{
  dtc_serial_info_rx_init();
  dtc_serial_info_rx_reset();

  for (int i = 0; i < serialTxBits; i++) {
    serialTxLevel[i] = (pin_cfg_input & 0xFF);
    serialTxDebug[i] = ((i % 2 == 0) ? pin_cfg_output_high : pin_cfg_output_low) & 0xFF;
  }

  for (int i = 0; i < serialRxBits; i++) {
    serialRxLevel[i] = 0xff;
    serialRxDebug[i] = ((i % 2 == 0) ? pin_cfg_output_high : pin_cfg_output_low) & 0xFF;
  }
}

void
HalfDuplexSerialCore::elc_link(int port)
{
  elc_event_t event = ELC_EVENT_NONE;
  static const elc_cfg_t elc_cfg = { ELC_EVENT_NONE }; // can we use g_elc_cfg??

  R_ELC_Open(&elc_ctrl, &elc_cfg);
  R_ELC_Disable(&elc_ctrl);
  switch (port) {
    case 1:
      event = ELC_EVENT_IOPORT_EVENT_1;
      break;
    case 2:
      event = ELC_EVENT_IOPORT_EVENT_2;
      break;
    case 3:
      event = ELC_EVENT_IOPORT_EVENT_3;
      break;
    case 4:
      event = ELC_EVENT_IOPORT_EVENT_4;
      break;
    default:
      event = ELC_EVENT_NONE;
      break;
  }
  R_ELC_LinkSet(&elc_ctrl, ELC_PERIPHERAL_GPT_A, event);
  R_ELC_Enable(&elc_ctrl);
}

void
HalfDuplexSerialCore::dtc_serial_info_tx_init()
{
  transfer_info_t *infop = dtc_info;
  static const int half = gpt_reg->GTPR_b.GTPR / 2;
  int bsp_port = bspPin >> IOPORT_PRV_PORT_OFFSET;
  int bsp_pinOffset = bspPin & BSP_IO_PRV_8BIT_MASK;
  void *REG_PFS = (void *)(&R_PFS->PORT[bsp_port].PIN[bsp_pinOffset].PmnPFS_BY);

  // GTIO pins (CHANNEL A)
  infop->transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->p_dest = REG_PFS;
  infop->transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED;
  infop->transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_END;
  infop->transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE; // unused
  infop->transfer_settings_word_b.size = TRANSFER_SIZE_1_BYTE;
  infop->transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL;
  infop->transfer_settings_word_b.irq = TRANSFER_IRQ_END;
  infop->num_blocks = 0; // unused.
  infop++;

  // stop timer
  infop->transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->p_dest = (void *)&(gpt_reg->GTSTP);
  infop->transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->p_src = (void *)&(gpt_stopVal);
  infop->transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_END;
  infop->transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE; // unused
  infop->transfer_settings_word_b.size = TRANSFER_SIZE_4_BYTE;
  infop->transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL;
  infop->transfer_settings_word_b.irq = TRANSFER_IRQ_END;
  infop->num_blocks = 0; // unused.
  infop++;

  // clear timer
  infop->transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->p_dest = (void *)&(gpt_reg->GTCNT);
  infop->transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->p_src = (void *)&(half);
  infop->transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_DISABLED;
  infop->transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE; // unused
  infop->transfer_settings_word_b.size = TRANSFER_SIZE_4_BYTE;
  infop->transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL;
  infop->transfer_settings_word_b.irq = TRANSFER_IRQ_END;
  infop->num_blocks = 0; // unused.
  infop++;

  assert(infop <= &dtc_info[dtc_info_len]);
}

void
HalfDuplexSerialCore::dtc_serial_info_tx_reset()
{
  transfer_info_t *infop = dtc_info;
  uint16_t bits = serialTxBits;

  // update GTIO state
  infop->p_src = &(serialTxLevel[0]);
  infop->length = bits;
  infop++;

  // stop timer
  infop->length = 1;
  infop++;

  // clear timer
  infop->length = 1;
  infop++;

  assert(infop <= &dtc_info[dtc_info_len]);
}

void
HalfDuplexSerialCore::dtc_serial_info_rx_init()
{
  transfer_info_t *infop = dtc_info;
  static const int half = gpt_reg->GTPR_b.GTPR / 2;
  int bsp_port = bspPin >> IOPORT_PRV_PORT_OFFSET;
  int bsp_pinOffset = bspPin & BSP_IO_PRV_8BIT_MASK;
  void *REG_PFS = (void *)(&R_PFS->PORT[bsp_port].PIN[bsp_pinOffset].PmnPFS_BY);

  // GTIO pins
  infop->transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED;
  infop->transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->p_src = REG_PFS;
  infop->transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_END;
  infop->transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE; // unused
  infop->transfer_settings_word_b.size = TRANSFER_SIZE_1_BYTE;
  infop->transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL;
  infop->transfer_settings_word_b.irq = TRANSFER_IRQ_END;
  infop->num_blocks = 0; // unused.
  infop++;

  // stop timer
  infop->transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->p_dest = (void *)&(gpt_reg->GTSTP);
  infop->transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->p_src = (void *)&(gpt_stopVal);
  infop->transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_END;
  infop->transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE; // unused
  infop->transfer_settings_word_b.size = TRANSFER_SIZE_4_BYTE;
  infop->transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL;
  infop->transfer_settings_word_b.irq = TRANSFER_IRQ_END;
  infop->num_blocks = 0; // unused.
  infop++;

  // clear timer
  infop->transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->p_dest = (void *)&(gpt_reg->GTCNT);
  infop->transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->p_src = (void *)&(half);
  infop->transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_DISABLED;
  infop->transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE; // unused
  infop->transfer_settings_word_b.size = TRANSFER_SIZE_4_BYTE;
  infop->transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL;
  infop->transfer_settings_word_b.irq = TRANSFER_IRQ_END;
  infop->num_blocks = 0; // unused.
  infop++;

  assert(infop <= &dtc_info[dtc_info_len]);
}

void
HalfDuplexSerialCore::dtc_serial_info_rx_reset()
{
  transfer_info_t *infop = dtc_info;
  uint16_t bits = serialRxBits;

  // update GTIO state
  infop->p_dest = &(serialRxLevel[0]);
  infop->length = bits;
  infop++;

  // stop timer
  infop->length = 1;
  infop++;

  // clear timer
  infop->length = 1;
  infop++;

  assert(infop <= &dtc_info[dtc_info_len]);
}

void
HalfDuplexSerialCore::tx_abort()
{
  noInterrupts();
  serialTxBytes = 0;
  interrupts();
  while (tx_busy) {
    yield();
  }
}

bool
HalfDuplexSerialCore::tx_serial_restart()
{
  load_serial_frame(CHANNEL_A);
  load_serial_frame(CHANNEL_B);
  serialTxPtr++;
  serialTxBytes--;

  dtc_serial_info_tx_reset();
  R_DTC_Enable(&dtc_ctrl);

  tx_busy = true;
  fsp_timer.start();
}

bool
HalfDuplexSerialCore::tx_serial_start(const uint8_t *data, size_t len)
{
  tx_abort();
 	R_DTC_Disable(&dtc_ctrl);
  tx_serial = true;
  rx_serial = false;

  serialTxPtr = data;
  serialTxBytes = len;

  dtc_serial_info_tx_init();

  gpt_reg->GTSSR_b.SSELCA = 0;

  return tx_serial_restart();
}

bool
HalfDuplexSerialCore::rx_serial_restart(bool initial)
{
  if (!initial) {
    #define RxLevelHigh(x) (((serialRxLevel[x]) & 0x2) != 0)

    if (RxLevelHigh(0)) {
      rx_serial_bad_frames++;
    }
    else if (!RxLevelHigh(9)) {
      rx_serial_bad_frames++;
    }
    else {
      rx_serial_good_frames++;

      for (int i = 1; i < 9; i++) {
        (*serialRxFIFO_IN) >>= 1;
        (*serialRxFIFO_IN) |= RxLevelHigh(i) ? (0x80): 0;
      }
      if (++serialRxFIFO_IN >= &serialRxFIFO[serialRxFIFOLen]) {
        serialRxFIFO_IN = serialRxFIFO;
      }
      if (++serialRxBytes > serialRxFIFOLen) {
        serialRxBytes = serialRxFIFOLen;
        serialRxFIFO_OUT = serialRxFIFO_IN;
      }
    }
  }

  gpt_reg->GTCNT_b.GTCNT = gpt_reg->GTPR_b.GTPR / 2;
  dtc_serial_info_rx_reset();
  R_DTC_Enable(&dtc_ctrl);
}

bool
HalfDuplexSerialCore::rx_serial_start()
{
  tx_abort();
	R_DTC_Disable(&dtc_ctrl);
  tx_serial = false;
  rx_serial = true;

  serialRxFIFO_IN = serialRxFIFO_OUT = serialRxFIFO;
  serialRxBytes = 0;

  dtc_serial_info_rx_init();

  gpt_reg->GTSSR_b.SSELCA = 1;

  return rx_serial_restart(true);
}

/*
Public
*/
HalfDuplexSerialCore::HalfDuplexSerialCore(FspTimer &timer, dtc_instance_ctrl_t &dtc, transfer_info_t *info) : fsp_timer(timer), dtc_ctrl(dtc), dtc_info(info)
{
	int ch = timer.get_channel();

	gpt_reg = (R_GPT0_Type *)((unsigned long)R_GPT0_BASE + (unsigned long)(0x0100 * ch));
	gpt_stopVal = 0x1 << ch;
}

HalfDuplexSerialCore::~HalfDuplexSerialCore()
{

}

void
HalfDuplexSerialCore::begin(TimerPWMChannel_t ch,  pin_size_t pin, uint32_t bps)
{
	this->channel = ch;
	this->digitalPin = pin;
	this->bspPin = digitalPinToBspPin(pin);
  this->bitRate = bps;

	pinPeripheral(pin, pin_cfg_input); // XXX: save current state.

	// Start serial communication @ 1 start bit, 1 stop bit, LSB first, 8 data bits.
	gpt_serial_init();
	dtc_serial_init();

	elc_link((bspPin >> IOPORT_PRV_PORT_OFFSET));

	rx_serial_start();
}

void
HalfDuplexSerialCore::end()
{
	// stop receiver.
	elc_link(-1);
	pinPeripheral(this->digitalPin, pin_cfg_output_low);
	rx_serial = false;
	tx_serial = false;

	// ensure GPT and DTC stopped.
	fsp_timer.stop();
	fsp_timer.reset();
	R_DTC_Disable(&dtc_ctrl);
}

size_t
HalfDuplexSerialCore::write(uint8_t c)
{
  tx_serial_start(&c, sizeof(c));
  while (tx_busy) {
    yield();
  }
  return sizeof(c);
}

size_t
HalfDuplexSerialCore::write(const uint8_t *buffer, size_t len)
{
  // buffering ???
  tx_serial_start(buffer, len);
  while (serialTxBytes > 0) {
    yield();
  }
  while (tx_busy) {
    yield();
  }
  return len;
}

int
HalfDuplexSerialCore::availableForWrite()
{
	return 0;
}

void
HalfDuplexSerialCore::flush()
{
	noInterrupts();
	serialRxFIFO_IN = serialRxFIFO_OUT = serialRxFIFO;
	serialRxBytes = 0;
	serialTxBytes = 0;
	interrupts();
	serialRxBuff_OUT = serialRxBuff;
	serialRxBuff_Remain = 0;

	while (tx_busy) {
		yield();
	}

	return;
}

int
HalfDuplexSerialCore::available()
{
	int bytes;

	noInterrupts();
	bytes = serialRxBytes;
	interrupts();
	bytes += serialRxBuff_Remain;

	return bytes;
}

int
HalfDuplexSerialCore::read()
{
	if (serialRxBuff_Remain == 0)
	{
		// flush FIFO.
		noInterrupts();
		if (serialRxBytes == 0)
		{
			interrupts();
			return -1;
		}
		for (int i = 0; i < serialRxBytes; i++)
		{
			serialRxBuff[i] = *serialRxFIFO_OUT++;
			if (serialRxFIFO_OUT >= &serialRxFIFO[serialRxFIFOLen])
			{
				serialRxFIFO_OUT = serialRxFIFO;
			}
		}
		serialRxBuff_Remain = serialRxBytes;
		serialRxFIFO_IN = serialRxFIFO_OUT = serialRxFIFO;
		serialRxBytes = 0;
		interrupts();
		serialRxBuff_OUT = serialRxBuff;
	}

	serialRxBuff_Remain--;
	return (int)*serialRxBuff_OUT++;
}

int
HalfDuplexSerialCore::peek()
{
	if (serialRxBuff_Remain > 0) {
		return (int)*serialRxBuff_OUT;
	}
	uint8_t byte;
	noInterrupts();
	if (serialRxBytes <= 0) {
		interrupts();
		return -1;
	}
	byte = *serialRxFIFO_OUT;
	interrupts();

	return (int)byte;
}
