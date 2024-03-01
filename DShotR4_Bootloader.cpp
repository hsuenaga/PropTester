#include "DShotR4.h"

#undef TX_DEBUG_LOW

void
DShotR4::tx_serial_complete(timer_callback_args_t (*arg))
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
DShotR4::rx_serial_complete(timer_callback_args_t (*arg))
{
  rx_serial_detect++;
  rx_serial_restart(false);

  return;
}

void
DShotR4::load_serial_frame(TimerPWMChannel_t ch)
{
  assert(tx_busy == false);
  assert(serialTxBits >= 12);

  uint8_t *p = serialTxLevel[ch];

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
DShotR4::gpt_serial_init()
{
  float default_duty = 50.0;
  float freqHz = (float)(19200); // 19200 bps

  fsp_timer.set_frequency(freqHz); // this fucntion starts timer automatically.
  fsp_timer.stop();
  fsp_timer.set_duty_cycle(default_duty, CHANNEL_A);
  fsp_timer.set_duty_cycle(default_duty, CHANNEL_B);
  fsp_timer.reset();
}

void
DShotR4::dtc_serial_init(void)
{
  dtc_serial_info_rx_init();
  dtc_serial_info_rx_reset();

  for (int i = 0; i < serialTxBits; i++) {
    serialTxLevel[0][i] = serialTxLevel[1][i] = (pin_cfg_input & 0xFF);
    #ifdef TX_DEBUG_LOW
    serialTxDebug[i] = pin_cfg_output_low & 0xFF;
    #else
    serialTxDebug[i] = ((i % 2 == 0) ? pin_cfg_output_high : pin_cfg_output_low) & 0xFF;
    #endif
  }

  for (int i = 0; i < serialRxBits; i++) {
    serialRxLevel[CHANNEL_A][i] = serialRxLevel[CHANNEL_B][i] = 0xff;
    serialRxDebug[i] = ((i % 2 == 0) ? pin_cfg_output_high : pin_cfg_output_low) & 0xFF;
  }
}

void
DShotR4::dtc_serial_info_tx_init()
{
  transfer_info_t *infop = dtc_info;
  static const int zero = 0;
  int bsp_portA = bspPinA >> IOPORT_PRV_PORT_OFFSET;
  int bsp_portB = bspPinB >> IOPORT_PRV_PORT_OFFSET;
  int bsp_pinOffsetA = bspPinA & BSP_IO_PRV_8BIT_MASK;
  int bsp_pinOffsetB = bspPinB & BSP_IO_PRV_8BIT_MASK;
  void *REG_A = (void *)(&R_PFS->PORT[bsp_portA].PIN[bsp_pinOffsetA].PmnPFS_BY);
  void *REG_B = (void *)(&R_PFS->PORT[bsp_portB].PIN[bsp_pinOffsetB].PmnPFS_BY);

  // GTIO pins (CHANNEL A)
  infop->transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->p_dest = REG_A;
  infop->transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED;
  infop->transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_EACH;
  infop->transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE; // unused
  infop->transfer_settings_word_b.size = TRANSFER_SIZE_1_BYTE;
  infop->transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL;
  infop->transfer_settings_word_b.irq = TRANSFER_IRQ_END;
  infop->num_blocks = 0; // unused.
  infop++;

  // GTIO pins (CHANNEL B)
  infop->transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->p_dest = REG_B;
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
  infop->p_src = (void *)&(this->gpt_ChannelRegVal);
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
  infop->p_src = (void *)&(zero);
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
DShotR4::dtc_serial_info_tx_reset()
{
  transfer_info_t *infop = dtc_info;
  uint16_t bits = serialTxBits;

  // update GTIO state / ChannelA
  #ifdef DEBUG_CHANNEL_A
  infop->p_src = &(serialTxDebug[0]);
  #else
  infop->p_src = &(serialTxLevel[CHANNEL_A][0]);
  #endif
  infop->length = bits;
  infop++;

  // update GTIO state / ChannelB
  infop->p_src = &(serialTxLevel[CHANNEL_B][0]);
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
DShotR4::dtc_serial_info_rx_init()
{
  transfer_info_t *infop = dtc_info;
  static const int half = gpt_reg->GTPR_b.GTPR / 2;
  int bsp_portA = bspPinA >> IOPORT_PRV_PORT_OFFSET;
  int bsp_portB = bspPinB >> IOPORT_PRV_PORT_OFFSET;
  int bsp_pinOffsetA = bspPinA & BSP_IO_PRV_8BIT_MASK;
  int bsp_pinOffsetB = bspPinB & BSP_IO_PRV_8BIT_MASK;
  void *REG_A = (void *)(&R_PFS->PORT[bsp_portA].PIN[bsp_pinOffsetA].PmnPFS_BY);
  void *REG_B = (void *)(&R_PFS->PORT[bsp_portB].PIN[bsp_pinOffsetB].PmnPFS_BY);

  // GTIO pins (CHANNEL A)
  infop->transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED;
  infop->transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->p_src = REG_A;
  infop->transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_EACH;
  infop->transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE; // unused
  infop->transfer_settings_word_b.size = TRANSFER_SIZE_1_BYTE;
  infop->transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL;
  infop->transfer_settings_word_b.irq = TRANSFER_IRQ_END;
  infop->num_blocks = 0; // unused.
#ifdef DEBUG_CHANNEL_A
  infop->transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->p_dest = REG_A;
  infop->transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED;
#endif
  infop++;

  // GTIO pins (CHANNEL B)
  infop->transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED;
  infop->transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->p_src = REG_B;
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
  infop->p_src = (void *)&(this->gpt_ChannelRegVal);
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
DShotR4::dtc_serial_info_rx_reset()
{
  transfer_info_t *infop = dtc_info;
  uint16_t bits = serialRxBits;

  // update GTIO state / ChannelA
  #ifdef DEBUG_CHANNEL_A
  infop->p_src = &(serialRxDebug[0]);
  #else
  infop->p_dest = &(serialRxLevel[CHANNEL_A][0]);
  #endif
  infop->length = bits;
  infop++;


  // update GTIO state / ChannelB
  infop->p_dest = &(serialRxLevel[CHANNEL_B][0]);
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

bool
DShotR4::tx_serial_restart()
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
DShotR4::tx_serial_start(const uint8_t *data, size_t len)
{
  while (tx_busy) {
    yield();
  }

  gpt_reg->GTSSR_b.SSELCA = 0;

  dtc_serial_info_tx_init();

  tx_serial = true;
  rx_serial = false;

  serialTxPtr = data;
  serialTxBytes = len;

  return tx_serial_restart();
}

bool
DShotR4::rx_serial_restart(bool initial)
{
  if (!initial) {
    // XXX: channel selection.
    #define RxLevelHigh(x) (((serialRxLevel[CHANNEL_B][x]) & 0x2) != 0)

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
DShotR4::rx_serial_start()
{
  tx_serial = false;
  rx_serial = true;

  serialRxFIFO_IN = serialRxFIFO_OUT = serialRxFIFO;
  serialRxBytes = 0;

  dtc_serial_info_rx_init();
  gpt_reg->GTSSR_b.SSELCA = 1;

  return rx_serial_restart(true);
}

bool
DShotR4::bl_enter()
{
  suspend();

  // disable GPIO/GTIO while setup.
  if (gpt_pwmChannelA_enable) {
      pinPeripheral(gpt_pwmPinA, pin_cfg_output_high);
  }
  if (gpt_pwmChannelB_enable) {
      pinPeripheral(gpt_pwmPinB, pin_cfg_output_high);
  }

  // Start serial communication @ 19200 bps, 1 start bit, 1 stop bit, LSB first, 8 databits.
  // bit speed: 52 us
  gpt_serial_init();
  gpt_reg->GTIOR = gtioHigh.gtior; // note: GTIO is disabled by pin config.
  dtc_serial_init();

  // ensure GPIO input mode.
  if (gpt_pwmChannelA_enable) {
      pinPeripheral(gpt_pwmPinA, pin_cfg_input);
  }
  if (gpt_pwmChannelB_enable) {
      pinPeripheral(gpt_pwmPinB, pin_cfg_input);
  }


  delay(1000); // wait for signal detection timeout.

  elc_link(bspPinA >> IOPORT_PRV_PORT_OFFSET);
  rx_serial_start();

  return true;
}

bool
DShotR4::bl_exit()
{
  // XXX: send exit commnad(0) to BLHeli.

  // stop receiver.
  rx_serial = false;
  tx_serial = false;
  elc_link(-1);

  // restore waveform transmission mode.
  R_DTC_Close(&dtc_ctrl);
  dtc_dshot_init();
  gpt_reg->GTIOR = gtioStop.gtior;
  gpt_dshot_init();

  // ensure GTIO output mode.
  if (gpt_pwmChannelA_enable) {
      pinPeripheral(gpt_pwmPinA, pin_cfg_output_gpt);
  }
  if (gpt_pwmChannelB_enable) {
      pinPeripheral(gpt_pwmPinB, pin_cfg_output_gpt);
  }

  return true;
}

bool
DShotR4::bl_open()
{
  tx_serial_start(blheli_signature, sizeof(blheli_signature));

  while (serialTxBytes > 0) {
    yield();
  }

  return true;
}

int
DShotR4::bl_read()
{
  if (serialRxBuff_Remain == 0) {
    // flush FIFO.
    noInterrupts();
    if (serialRxBytes == 0) {
      interrupts();
      return -1;
    }
    for (int i = 0; i < serialRxBytes; i++) {
      serialRxBuff[i] = *serialRxFIFO_OUT++;
      if (serialRxFIFO_OUT >= &serialRxFIFO[serialRxFIFOLen]) {
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
DShotR4::bl_peek()
{
  int bytes;

  noInterrupts();
  bytes = serialRxBytes;
  interrupts();
  bytes += serialRxBuff_Remain;

  return bytes;
}

int
DShotR4::bl_flush()
{
  int bytes;

  noInterrupts();
  bytes = serialRxBytes;
  serialRxFIFO_IN = serialRxFIFO_OUT = serialRxFIFO;
  serialRxBytes = 0;
  interrupts();

  return bytes;
}

int
DShotR4::bl_write(uint8_t data)
{
  tx_serial_start(&data, sizeof(data));
}

