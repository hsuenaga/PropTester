#include "DShotR4.h"

void
DShotR4::tx_serial_complete(timer_callback_args_t (*arg))
{
  tx_busy = false;
  tx_serial_success++;
//  gpt_reg->GTSTP = gpt_ChannelRegVal;
//  gpt_reg->GTCLR = gpt_ChannelRegVal;
  if (serialTxBytes > 0) {
    tx_serial_restart();
    return;
  }
  if (gpt_pwmChannelA_enable) {
      pinPeripheral(gpt_pwmPinA, pin_cfg_input);
  }
  if (gpt_pwmChannelB_enable) {
      pinPeripheral(gpt_pwmPinB, pin_cfg_input);
  }

  return;
}

void
DShotR4::load_serial_frame()
{
  assert(tx_busy == false);
  assert(serialBits >= 12);

  uint32_t *p = serialState;

  *p++ = gtioHigh.gtior; // idle
  *p++ = gtioLow.gtior; // start bit
  for (int i = 0; i < 8; i++) {
    bool bit = ((*serialTxData >> i) & 0x1) == 0x1;
    *p++ = bit ? gtioHigh.gtior : gtioLow.gtior;
  }
  *p++ = gtioHigh.gtior;
  *p++ = gtioHigh.gtior; // idle
}

void
DShotR4::gpt_serial_reinit()
{
  float default_duty = 50.0;
  float freqHz = (float)(19200); // 19200 bps

  fsp_timer.set_frequency(freqHz); // this fucntion starts timer automatically.
  fsp_timer.set_duty_cycle(default_duty, CHANNEL_A);
  fsp_timer.set_duty_cycle(default_duty, CHANNEL_B);
  fsp_timer.stop();
  fsp_timer.reset();
}

void
DShotR4::dtc_serial_init(void)
{
  assert(GPT_IRQn != FSP_INVALID_VECTOR);

  dtc_serial_info_tx_init(dtc_serial_info);
  dtc_cfg.p_info = dtc_serial_info;

  dtc_extcfg.activation_source = GPT_IRQn;
  dtc_cfg.p_extend = &dtc_extcfg;

  fsp_err_t err = R_DTC_Open(&dtc_ctrl, &dtc_cfg);
  assert(FSP_SUCCESS == err);
}

void
DShotR4::dtc_serial_info_tx_init(transfer_info_t *info)
{
  transfer_info_t *infop = info;
  static const int zero = 0;

  // GTIO pins
  infop->transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->p_dest = (void *)&(gpt_reg->GTIOR);
  infop->transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED;
  infop->transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_END;
  infop->transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE; // unused
  infop->transfer_settings_word_b.size = TRANSFER_SIZE_4_BYTE;
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

  dtc_serial_info_tx_reset(info);
}

void
DShotR4::dtc_serial_info_tx_reset(transfer_info_t *info)
{
  transfer_info_t *infop = info;
  uint16_t bits = serialBits - 1;

  // update GTIO state
  infop->p_src = &(serialState[1]);
  infop->length = bits;
  infop++;

  // stop timer
  infop->length = 1;
  infop++;

  // clear timer
  infop->length = 1;
  infop++;
}

bool
DShotR4::tx_serial_restart()
{
  load_serial_frame();
  serialTxData++;
  serialTxBytes--;

  gpt_reg->GTIOR = serialState[0];

  dtc_serial_info_tx_reset(dtc_serial_info);
  R_DTC_Reconfigure(&dtc_ctrl, dtc_serial_info);
  R_DTC_Enable(&dtc_ctrl);

  tx_busy = true;
  fsp_timer.start();
}

bool
DShotR4::tx_serial_start(const uint8_t *data, size_t len)
{
  assert(fsp_timer.is_opened() == true);

  cancel_execution();

  serialTxData = data;
  serialTxBytes = len;

  if (gpt_pwmChannelA_enable) {
      pinPeripheral(gpt_pwmPinA, pin_cfg_output);
  }
  if (gpt_pwmChannelB_enable) {
      pinPeripheral(gpt_pwmPinB, pin_cfg_output);
  }

  return tx_serial_restart();
}

bool
DShotR4::bl_enter()
{
  suspend();

  // disable GTIO while setup.
  if (gpt_pwmChannelA_enable) {
      pinPeripheral(gpt_pwmPinA, pin_cfg_output_high);
  }
  if (gpt_pwmChannelB_enable) {
      pinPeripheral(gpt_pwmPinB, pin_cfg_output_high);
  }
  
  // Start serial communication @ 19200 bps, 1 start bit, 1 stop bit, LSB first, 8 databits.
  // bit speed: 52 us
  gpt_serial_reinit();
  gpt_reg->GTIOR = gtioHigh.gtior;

  // ensure GTIO output mode.
  if (gpt_pwmChannelA_enable) {
      pinPeripheral(gpt_pwmPinA, pin_cfg_input);
  }
  if (gpt_pwmChannelB_enable) {
      pinPeripheral(gpt_pwmPinB, pin_cfg_input);
  }

  R_DTC_Close(&dtc_ctrl);
  dtc_serial_init();
  tx_serial = true;

  delay(1000); // wait for signal detection timeout.

  tx_serial_start(blheli_signature, sizeof(blheli_signature));

  return true;
}

bool
DShotR4::bl_exit()
{
  // XXX: send exit commnad(0) to BLHeli.

  // ensure output mode.
  if (gpt_pwmChannelA_enable) {
      pinPeripheral(gpt_pwmPinA, pin_cfg_output);
  }
  if (gpt_pwmChannelB_enable) {
      pinPeripheral(gpt_pwmPinB, pin_cfg_output);
  }

  // restore waveform transmission mode.
  tx_serial = false;
  R_DTC_Close(&dtc_ctrl);
  dtc_dshot_init();
  gpt_reg->GTIOR = gtioStop.gtior;
  gpt_dshot_reinit();
    
  return true;
}

int
DShotR4::bl_read()
{
}

int
DShotR4::bl_write(uint8_t data)
{
  tx_serial_start(&data, sizeof(data));
}

