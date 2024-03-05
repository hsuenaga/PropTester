#include "HalfDuplexSerial.h"

void HalfDuplexSerialCore::tx_serial_complete(timer_callback_args_t(*arg))
{
  tx_busy = false;
  Counter.tx_success++;
  tx_serial_restart(false);

  return;
}

void HalfDuplexSerialCore::rx_serial_complete(timer_callback_args_t(*arg))
{
  Counter.rx_detect++;
  rx_serial_restart(false);

  return;
}

void HalfDuplexSerialCore::gpt_init()
{
  float default_duty = 0.0;

  fspTimer.set_frequency((float)bitRate); // this function starts timer automatically.
  fspTimer.stop();
  fspTimer.set_duty_cycle(default_duty, CHANNEL_A);
  fspTimer.set_duty_cycle(default_duty, CHANNEL_B);
  gptReg->GTCNT_b.GTCNT = gptReg->GTPR_b.GTPR / 2;
}

void HalfDuplexSerialCore::elc_link(int port)
{
  elc_event_t event = ELC_EVENT_NONE;
  static const elc_cfg_t elc_cfg = {ELC_EVENT_NONE}; // can we use g_elc_cfg??

  R_ELC_Open(&elcCtrl, &elc_cfg);
  R_ELC_Disable(&elcCtrl);
  switch (port)
  {
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

void HalfDuplexSerialCore::dtc_init(void)
{
  dtc_info_rx_init();
  dtc_info_rx_reset();
}

void HalfDuplexSerialCore::dtc_info_tx_init()
{
  transfer_info_t *infop = dtcInfo;
  static uint32_t stop;

  stop = 0x1 << fspTimer.get_channel();

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

  // reload timer
  infop->transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->p_dest = (void *)&(gptReg->GTCNT);
  infop->transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->p_src = (void *)&(txIFG);
  infop->transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_DISABLED;
  infop->transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE; // unused
  infop->transfer_settings_word_b.size = TRANSFER_SIZE_4_BYTE;
  infop->transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL;
  infop->transfer_settings_word_b.irq = TRANSFER_IRQ_END;
  infop->num_blocks = 0; // unused.
  infop++;

  assert(infop <= &dtcInfo[dtcInfoLen]);
}

void HalfDuplexSerialCore::dtc_info_tx_reset()
{
  transfer_info_t *infop = dtcInfo;
  uint16_t bits = txBits;

  // update GTIO state
  infop->p_src = &(*txFIFO_OUT)[0];
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

void HalfDuplexSerialCore::dtc_info_rx_init()
{
  transfer_info_t *infop = dtcInfo;
  static uint32_t stop;
  static uint32_t half;

  stop = 0x1 << fspTimer.get_channel();
  half = gptReg->GTPR_b.GTPR / 2;

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

void HalfDuplexSerialCore::dtc_info_rx_reset()
{
  transfer_info_t *infop = dtcInfo;
  uint16_t bits = rxBits;
  static uint8_t discard[rxBits];
  uint8_t *buffer = rx_overflow ? &discard[0] : &(*rxFIFO_IN)[0];

  // update GTIO state
  infop->p_dest = buffer;
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

void HalfDuplexSerialCore::tx_encode(uint8_t byte, txPFSBY_t *pfsby)
{
  uint8_t *p = &(*pfsby)[0];

  *p++ = pinCfgOutputHigh & 0xFF; // idle
  *p++ = pinCfgOutputLow & 0xFF; // start bit
  for (int i = 0; i < 8; i++)
  {
    bool bit = ((byte >> i) & 0x1) == 0x1;
    *p++ = (bit ? pinCfgOutputHigh : pinCfgOutputLow) & 0xFF;
  }
  *p++ = pinCfgOutputHigh & 0xFF; // stop bit
  *p++ = pinCfgOutputHigh & 0xFF; // idle
}

void HalfDuplexSerialCore::tx_abort()
{
  noInterrupts();
  txFIFO_Bytes = 0;
  interrupts();
  while (tx_busy)
  {
    yield();
  }
  noInterrupts();
  txFIFO_IN = txFIFO_OUT = &txFIFO[0];
  txFIFO_Bytes = 0;
  interrupts();
}

bool HalfDuplexSerialCore::tx_serial_restart(bool initial)
{
  if (initial) {
    // send 1st bit
    txIFG = gptReg->GTPR - 1;
    gptReg->GTCNT = txIFG;
  }
  else {
    txFIFO_OUT++;
    if (txFIFO_OUT >= &txFIFO[txFIFOLen]) {
      txFIFO_OUT = &txFIFO[0];
    }
    txFIFO_Bytes--;
    if (txFIFO_Bytes <= 0) {
      return rx_serial_start();
    }
  }

  tx_busy = true;
  dtc_info_tx_reset();
  R_DTC_Enable(&dtcCtrl);
  fspTimer.start();
  return true;
}

bool HalfDuplexSerialCore::tx_serial_start()
{
  if (!rx_ready) {
    // Tx is already running.
    return true;
  }
  gptReg->GTSSR_b.SSELCA = 0;
  fspTimer.stop();
  R_DTC_Disable(&dtcCtrl);
  rx_ready = false;

  dtc_info_tx_init();

  pinPeripheral(bspPin, pinCfgOutputHigh);
  return tx_serial_restart(true);
}

int HalfDuplexSerialCore::rx_decode(rxPFSBY_t *pfsby)
{
  uint8_t v = 0;
  uint8_t *p = &(*pfsby)[0];

  // start bit must be low
  if ((p[0] & R_PFS_PORT_PIN_PmnPFS_PIDR_Msk))
  {
    return -1;
  }

  // stop bit must be high
  if (!(p[9] & R_PFS_PORT_PIN_PmnPFS_PIDR_Msk))
  {
    return -1;
  }

  for (int i = 1; i < 9; i++)
  {
    v >>= 1;
    v |= (p[i] & R_PFS_PORT_PIN_PmnPFS_PIDR_Msk) ? 0x80 : 0;
  }
  return (int)v;
}

bool HalfDuplexSerialCore::rx_serial_restart(bool initial)
{
  if (!initial && !rx_overflow)
  {
    rxFIFO_IN++;
    if (rxFIFO_IN >= &rxFIFO[rxFIFOLen])
    {
      rxFIFO_IN = &rxFIFO[0];
    }
    rxFIFO_Bytes++;
  }
  if (rxFIFO_Bytes >= rxFIFOLen)
  {
    // discard next frame.
    rx_overflow = true;
    Counter.rx_overflow++;
  }

  dtc_info_rx_reset();
  R_DTC_Enable(&dtcCtrl);
  gptReg->GTSSR_b.SSELCA = 1;

  return true;
}

bool HalfDuplexSerialCore::rx_serial_start()
{
  if (rx_ready)
  {
    return false;
  }

  tx_abort();
  R_DTC_Disable(&dtcCtrl);
  rx_ready = true;

  dtc_info_rx_init();

  gptReg->GTCNT = gptReg->GTPR_b.GTPR / 2;

  pinPeripheral(bspPin, pinCfgInput);
  return rx_serial_restart(true);
}

/*
Public
*/
HalfDuplexSerialCore::HalfDuplexSerialCore(FspTimer &timer, dtc_instance_ctrl_t &dtc, transfer_info_t *info, size_t infoLen)
    : fspTimer(timer),
      dtcCtrl(dtc),
      dtcInfo(info),
      dtcInfoLen(infoLen),
      txFIFO_IN(&txFIFO[0]),
      txFIFO_OUT(&txFIFO[0]),
      rxFIFO_IN(&rxFIFO[0]),
      rxFIFO_OUT(&rxFIFO[0]),
      rx_overflow(false)
{
}

HalfDuplexSerialCore::~HalfDuplexSerialCore()
{
}

void HalfDuplexSerialCore::overflow_interrupt(timer_callback_args_t(*arg))
{
  if (tx_busy)
  {
    return tx_serial_complete(arg);
  }

  if (rx_ready)
  {
    return rx_serial_complete(arg);
  }

  Counter.spurious_interrupts++;
  return;
}

void HalfDuplexSerialCore::begin(TimerPWMChannel_t ch, pin_size_t pin, uint32_t bps)
{
  this->channel = ch;
  this->digitalPin = pin;
  this->bspPin = digitalPinToBspPin(pin);
  this->bitRate = bps;
  this->bspPort = bspPin >> IOPORT_PRV_PORT_OFFSET;
  this->bspPinOffset = bspPin & BSP_IO_PRV_8BIT_MASK;
  this->gptReg = (R_GPT0_Type *)((unsigned long)R_GPT0_BASE + (unsigned long)(0x0100 * fspTimer.get_channel()));

  pinCfgSave = R_PFS->PORT[bspPort].PIN[bspPinOffset].PmnPFS;
  pinCfgSave &= ~(R_PFS_PORT_PIN_PmnPFS_PIDR_Msk);
  pinPeripheral(pin, pinCfgInput); // XXX: save current state.

  // Start serial communication
  //  1 start bit, 1 stop bit, LSB first, 8 data bits.
  gpt_init();
  dtc_init();

  elc_link((bspPin >> IOPORT_PRV_PORT_OFFSET));

  rx_serial_start();
}

void HalfDuplexSerialCore::end()
{
  // stop receiver.
  elc_link(-1);
  pinPeripheral(this->digitalPin, pinCfgSave);

  // ensure GPT and DTC stopped.
  fspTimer.stop();
  fspTimer.reset();
  R_DTC_Disable(&dtcCtrl);

  tx_busy = false;
  rx_ready = false;
}

size_t
HalfDuplexSerialCore::write(uint8_t c)
{
  while (txFIFO_Bytes >= txFIFOLen) {
    yield();
  }

  tx_encode(c, txFIFO_IN++);
  if (txFIFO_IN >= &txFIFO[txFIFOLen])
  {
    txFIFO_IN = &txFIFO[0];
  }

  noInterrupts();
  txFIFO_Bytes++;
  interrupts();

  (void)tx_serial_start();

  return sizeof(c);
}

int HalfDuplexSerialCore::availableForWrite()
{
  return txFIFOLen - txFIFO_Bytes;
}

void HalfDuplexSerialCore::flush()
{
  while (txFIFO_Bytes > 0)
  {
    yield();
  }
}

int HalfDuplexSerialCore::available()
{
  return rxFIFO_Bytes;
}

int HalfDuplexSerialCore::read()
{
  if (rxFIFO_Bytes <= 0)
  {
    return -1;
  }

  int byte = rx_decode(rxFIFO_OUT);
  rxFIFO_OUT++;
  if (rxFIFO_OUT >= &rxFIFO[rxFIFOLen])
  {
    rxFIFO_OUT = &rxFIFO[0];
  }
  noInterrupts();
  rxFIFO_Bytes--;
  rx_overflow = false;
  interrupts();

  if (byte < 0) {
    Counter.rx_bad_frames++;
  }

  return byte;
}

int HalfDuplexSerialCore::peek()
{
  if (rxFIFO_Bytes <= 0)
  {
    return -1;
  }

  return rx_decode(rxFIFO_OUT);
}
