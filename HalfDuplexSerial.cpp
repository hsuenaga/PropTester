#include "HalfDuplexSerial.h"

void
HalfDuplexSerialCore::tx_serial_complete(timer_callback_args_t (*arg))
{
  tx_busy = false;
  Counter.tx_success++;
  if (txBytes > 0) {
    tx_serial_restart();
    return;
  }
  rx_serial_start();

  return;
}

void
HalfDuplexSerialCore::rx_serial_complete(timer_callback_args_t (*arg))
{
  Counter.rx_detect++;
  rx_serial_restart(false);

  return;
}

void
HalfDuplexSerialCore::gpt_serial_init()
{
  float default_duty = 0.0;

  fspTimer.set_frequency((float)bitRate); // this function starts timer automatically.
  fspTimer.stop();
  fspTimer.set_duty_cycle(default_duty, CHANNEL_A);
  fspTimer.set_duty_cycle(default_duty, CHANNEL_B);
  gptReg->GTCNT_b.GTCNT = gptReg->GTPR_b.GTPR / 2;
}

void
HalfDuplexSerialCore::elc_link(int port)
{
  elc_event_t event = ELC_EVENT_NONE;
  static const elc_cfg_t elc_cfg = { ELC_EVENT_NONE }; // can we use g_elc_cfg??

  R_ELC_Open(&elcCtrl, &elc_cfg);
  R_ELC_Disable(&elcCtrl);
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
  R_ELC_LinkSet(&elcCtrl, ELC_PERIPHERAL_GPT_A, event);
  R_ELC_Enable(&elcCtrl);
}

void
HalfDuplexSerialCore::dtc_serial_init(void)
{
  dtc_serial_info_rx_init();
  dtc_serial_info_rx_reset();

  for (int i = 0; i < txBits; i++) {
    txPFSBY[i] = (pinCfgInput & 0xFF);
    txDebug[i] = ((i % 2 == 0) ? pinCfgOutputHigh : pinCfgOutputLow) & 0xFF;
  }

  for (int i = 0; i < rxBits; i++) {
    rxPFSBY[i] = 0xff;
    rxDebug[i] = ((i % 2 == 0) ? pinCfgOutputHigh : pinCfgOutputLow) & 0xFF;
  }
}

void
HalfDuplexSerialCore::dtc_serial_info_tx_init()
{
  transfer_info_t *infop = dtcInfo;
  static uint32_t half = gptReg->GTPR_b.GTPR / 2;
	static uint32_t stop = 0x1 << fspTimer.get_channel();

  // GTIO pins (CHANNEL A)
  infop->transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->p_dest = (void *)&(R_PFS->PORT[bspPort].PIN[bspPinOffset].PmnPFS_BY);
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
  infop->p_dest = (void *)&(gptReg->GTSTP);
  infop->transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->p_src = (void *)&(stop);
  infop->transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_END;
  infop->transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE; // unused
  infop->transfer_settings_word_b.size = TRANSFER_SIZE_4_BYTE;
  infop->transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL;
  infop->transfer_settings_word_b.irq = TRANSFER_IRQ_END;
  infop->num_blocks = 0; // unused.
  infop++;

  // clear timer
  infop->transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->p_dest = (void *)&(gptReg->GTCNT);
  infop->transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->p_src = (void *)&(half);
  infop->transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_DISABLED;
  infop->transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE; // unused
  infop->transfer_settings_word_b.size = TRANSFER_SIZE_4_BYTE;
  infop->transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL;
  infop->transfer_settings_word_b.irq = TRANSFER_IRQ_END;
  infop->num_blocks = 0; // unused.
  infop++;

  assert(infop <= &dtcInfo[dtcInfoLen]);
}

void
HalfDuplexSerialCore::dtc_serial_info_tx_reset()
{
  transfer_info_t *infop = dtcInfo;
  uint16_t bits = txBits;

  // update GTIO state
  infop->p_src = &(txPFSBY[0]);
  infop->length = bits;
  infop++;

  // stop timer
  infop->length = 1;
  infop++;

  // clear timer
  infop->length = 1;
  infop++;

  assert(infop <= &dtcInfo[dtcInfoLen]);
}

void
HalfDuplexSerialCore::dtc_serial_info_rx_init()
{
  transfer_info_t *infop = dtcInfo;
  static uint32_t half = gptReg->GTPR_b.GTPR / 2;
	static uint32_t stop = 0x1 << fspTimer.get_channel();

  // GTIO pins
  infop->transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED;
  infop->transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->p_src = (void *)&(R_PFS->PORT[bspPort].PIN[bspPinOffset].PmnPFS_BY);
  infop->transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_END;
  infop->transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE; // unused
  infop->transfer_settings_word_b.size = TRANSFER_SIZE_1_BYTE;
  infop->transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL;
  infop->transfer_settings_word_b.irq = TRANSFER_IRQ_END;
  infop->num_blocks = 0; // unused.
  infop++;

  // stop timer
  infop->transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->p_dest = (void *)&(gptReg->GTSTP);
  infop->transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->p_src = (void *)&(stop);
  infop->transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_END;
  infop->transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE; // unused
  infop->transfer_settings_word_b.size = TRANSFER_SIZE_4_BYTE;
  infop->transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL;
  infop->transfer_settings_word_b.irq = TRANSFER_IRQ_END;
  infop->num_blocks = 0; // unused.
  infop++;

  // clear timer
  infop->transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->p_dest = (void *)&(gptReg->GTCNT);
  infop->transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->p_src = (void *)&(half);
  infop->transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_DISABLED;
  infop->transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE; // unused
  infop->transfer_settings_word_b.size = TRANSFER_SIZE_4_BYTE;
  infop->transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL;
  infop->transfer_settings_word_b.irq = TRANSFER_IRQ_END;
  infop->num_blocks = 0; // unused.
  infop++;

  assert(infop <= &dtcInfo[dtcInfoLen]);
}

void
HalfDuplexSerialCore::dtc_serial_info_rx_reset()
{
  transfer_info_t *infop = dtcInfo;
  uint16_t bits = rxBits;

  // update GTIO state
  infop->p_dest = &(rxPFSBY[0]);
  infop->length = bits;
  infop++;

  // stop timer
  infop->length = 1;
  infop++;

  // clear timer
  infop->length = 1;
  infop++;

  assert(infop <= &dtcInfo[dtcInfoLen]);
}

void
HalfDuplexSerialCore::tx_encode()
{
  assert(tx_busy == false);
  assert(txBits >= 12);

  uint8_t *p = txPFSBY;

  *p++ = pinCfgInput; // idle
  *p++ = pinCfgOutputLow; // start bit
  for (int i = 0; i < 8; i++) {
    bool bit = ((*txPtr >> i) & 0x1) == 0x1;
    *p++ = (bit ? pinCfgOutputHigh : pinCfgOutputLow) & 0xFF;
  }
  *p++ = pinCfgOutputHigh; // stop bit
  *p++ = pinCfgInput; // idle
}

void
HalfDuplexSerialCore::tx_abort()
{
  noInterrupts();
  txBytes = 0;
  interrupts();
  while (tx_busy) {
    yield();
  }
}

bool
HalfDuplexSerialCore::tx_serial_restart()
{
  tx_encode();
  txPtr++;
  txBytes--;

  dtc_serial_info_tx_reset();
  R_DTC_Enable(&dtcCtrl);

  tx_busy = true;
  fspTimer.start();
}

bool
HalfDuplexSerialCore::tx_serial_start(const uint8_t *data, size_t len)
{
  tx_abort();
 	R_DTC_Disable(&dtcCtrl);
  tx_serial = true;
  rx_serial = false;

  txPtr = data;
  txBytes = len;

  dtc_serial_info_tx_init();

  gptReg->GTSSR_b.SSELCA = 0;

  return tx_serial_restart();
}

int
HalfDuplexSerialCore::rx_decode()
{
  uint8_t v = 0;

  // start bit must be low
  if ((rxPFSBY[0] & R_PFS_PORT_PIN_PmnPFS_PIDR_Msk)) {
    return -1;
  }

  // stop bit must be high
  if (!(rxPFSBY[9] & R_PFS_PORT_PIN_PmnPFS_PIDR_Msk)) {
    return -1;
  }

  for (int i = 1; i < 9; i++) {
    v >>= 1;
    v |= (rxPFSBY[i] & R_PFS_PORT_PIN_PmnPFS_PIDR_Msk) ? 0x80 : 0;
  }
  return (int)v;
}

bool
HalfDuplexSerialCore::rx_serial_restart(bool initial)
{
  if (!initial) {
    int v = rx_decode();

    if (v < 0) {
      Counter.rx_bad_frames++;
    }
    else if (rxFIFO_Bytes >= rxFIFOLen) {
      Counter.rx_overflow++;
    }
    else {
      Counter.rx_good_frames++;
      *rxFIFO_IN++ = (uint8_t)v;
      rxFIFO_Bytes++;

      if (rxFIFO_IN >= &rxFIFO[rxFIFOLen]) {
        rxFIFO_IN = rxFIFO;
      }
    }
  }

  dtc_serial_info_rx_reset();
  R_DTC_Enable(&dtcCtrl);

  return true;
}

bool
HalfDuplexSerialCore::rx_serial_start()
{
  if (rx_serial) {
    return false;
  }

  tx_abort();
	R_DTC_Disable(&dtcCtrl);
  tx_serial = false;
  rx_serial = true;

  dtc_serial_info_rx_init();

  gptReg->GTSSR_b.SSELCA = 1;

  return rx_serial_restart(true);
}

/*
Public
*/
HalfDuplexSerialCore::HalfDuplexSerialCore(FspTimer &timer, dtc_instance_ctrl_t &dtc, transfer_info_t *info) : fspTimer(timer), dtcCtrl(dtc), dtcInfo(info)
{
	int ch = timer.get_channel();

	gptReg = (R_GPT0_Type *)((unsigned long)R_GPT0_BASE + (unsigned long)(0x0100 * ch));
}

HalfDuplexSerialCore::~HalfDuplexSerialCore()
{

}

void
HalfDuplexSerialCore::overflow_interrupt(timer_callback_args_t (*arg))
{
  if (tx_serial) {
    return tx_serial_complete(arg);
  }
  else if (rx_serial) {
    return rx_serial_complete(arg);
  }
  Counter.spurious_interrupts++;
  return;
}

void
HalfDuplexSerialCore::begin(TimerPWMChannel_t ch,  pin_size_t pin, uint32_t bps)
{
	this->channel = ch;
	this->digitalPin = pin;
	this->bspPin = digitalPinToBspPin(pin);
  this->bitRate = bps;
  this->bspPort = bspPin >> IOPORT_PRV_PORT_OFFSET;
  this->bspPinOffset = bspPin & BSP_IO_PRV_8BIT_MASK;

  pinCfgSave = R_PFS->PORT[bspPort].PIN[bspPinOffset].PmnPFS;
  pinCfgSave &= ~(R_PFS_PORT_PIN_PmnPFS_PIDR_Msk);
	pinPeripheral(pin, pinCfgInput); // XXX: save current state.

	// Start serial communication
  //  1 start bit, 1 stop bit, LSB first, 8 data bits.
	gpt_serial_init();
	dtc_serial_init();

	elc_link((bspPin >> IOPORT_PRV_PORT_OFFSET));

  rxFIFO_IN = rxFIFO_OUT = rxFIFO;
  rxFIFO_Bytes = 0;

	rx_serial_start();
}

void
HalfDuplexSerialCore::end()
{
	// stop receiver.
	elc_link(-1);
	pinPeripheral(this->digitalPin, pinCfgSave);
	rx_serial = false;
	tx_serial = false;

	// ensure GPT and DTC stopped.
	fspTimer.stop();
	fspTimer.reset();
	R_DTC_Disable(&dtcCtrl);
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
  // do we want buffering behaviors???
  tx_serial_start(buffer, len);
  flush();

  return len;
}

int
HalfDuplexSerialCore::availableForWrite()
{
	return 0; // no buffering is supported at this time.
}

void
HalfDuplexSerialCore::flush()
{
  while (txBytes > 0) {
    yield();
  }
  while (tx_busy) {
    yield();
  }
}

int
HalfDuplexSerialCore::available()
{
	int bytes;

	noInterrupts();
	bytes = rxFIFO_Bytes;
	interrupts();

	return bytes;
}

int HalfDuplexSerialCore::read()
{
  uint8_t byte;

  noInterrupts();
  if (rxFIFO_Bytes <= 0)
  {
    interrupts();
    return -1;
  }
  byte = *rxFIFO_OUT++;
  rxFIFO_Bytes--;
  if (rxFIFO_OUT >= &rxFIFO[rxFIFOLen])
  {
    rxFIFO_OUT = rxFIFO;
  }
  interrupts();

  return (int)byte;
}

int
HalfDuplexSerialCore::peek()
{
	uint8_t byte;

	noInterrupts();
	if (rxFIFO_Bytes <= 0) {
		interrupts();
		return -1;
	}
	byte = *rxFIFO_OUT;
	interrupts();

	return (int)byte;
}
