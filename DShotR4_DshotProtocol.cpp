#include "DShotR4.h"

#include "bsp_api.h"
void
DShotR4::tx_dshot_complete(timer_callback_args_t (*arg))
{
  tx_success++;
  tx_busy = false;

  if (auto_restart) {
    if (auto_restart_count < 0) {
      // always restart
      tx_restart();
      return;
    }
    else if (auto_restart_count == 0) {
      // nothing to do.
      return;
    }
    else {
      // restart specified times.
      auto_restart_count--;
      tx_restart();
      return;
    }
  }

  return;
}

void
DShotR4::load_dshot_frame()
{
  // NEVER call while DTC is running.
  assert(tx_busy == false);

  for (int i = 0; i < 16; i++) {    
    bool bit;

    if (gpt_pwmChannelA_enable) {
      bit = ((nextFrame[0] << i) & 0x8000) != 0;
      waveform[0][i] = bit ? t1h_count : t0h_count;
#ifdef DEBUG_CHANNEL_A
      waveform[0][i] = period_count / 2;
#endif
    }
    if (gpt_pwmChannelB_enable) {
      bit = ((nextFrame[1] << i) & 0x8000) != 0;
      waveform[1][i] = bit ? t1h_count : t0h_count;
    }
  }

  nextFrameUpdate = false;
}

void
DShotR4::update_timer_counts(void)
{
  assert(fsp_timer.is_opened() == true);

  const struct dshot_params_t *param = &dshot_timing[dshot_type];

  period_count = fsp_timer.get_cfg()->period_counts;

  float t1h_ratio = param->t1h_us / param->bit_duration_us;
  float t0h_ratio = param->t0h_us / param->bit_duration_us;

  t1h_count = (uint32_t)ceil((float)period_count * t1h_ratio * tolerance_t1h);
  t0h_count = (uint32_t)floor((float)period_count * t0h_ratio * tolerance_t0h);
  stop_count = 0;
  dshot_ifg_us = (uint32_t)ceil(param->bit_duration_us * ifgBits);
}

void
DShotR4::gpt_dshot_init()
{
  float default_duty = 0.0;
  float freqHz = (float)(1.0 / dshot_timing[dshot_type].bit_duration_us * 1000.0 * 1000.0 * tolerance_hz);

  cancel_execution();

  fsp_timer.set_frequency(freqHz); // this function starts timer automaticcaly.
  fsp_timer.stop();
  fsp_timer.set_duty_cycle(default_duty, CHANNEL_A);
  fsp_timer.set_duty_cycle(default_duty, CHANNEL_B);
  fsp_timer.reset();

  // Initialize waveforms
  update_timer_counts();
  for (int i = 0; i < dshotBits; i++) {
    waveform[0][i] = waveform[1][i] = stop_count;
#ifdef DEBUG_CHANNEL_A
    waveform[0][i] = period_count / 2;
#endif
  }

  if (gpt_pwmChannelA_enable) {
      pinPeripheral(gpt_pwmPinA, pin_cfg_output_gpt);
  }
  if (gpt_pwmChannelB_enable) {
      pinPeripheral(gpt_pwmPinB, pin_cfg_output_gpt);
  }
}

void
DShotR4::dtc_dshot_info_init(transfer_info_t *info)
{
  transfer_info_t *infop = info;

  // transfer waveform (channel A)
  if (gpt_pwmChannelA_enable) {
    infop->transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED;
    infop->p_dest = (void *)&(gpt_reg->GTCCR[GTCCR_C]);
    infop->transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED;
    infop->transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_EACH;
    infop->transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE; // unused
    infop->transfer_settings_word_b.size = TRANSFER_SIZE_4_BYTE;
    infop->transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL;
    infop->transfer_settings_word_b.irq = TRANSFER_IRQ_END;
    infop->num_blocks = 0; // unused
    infop++;
  }

  // transfer waveform (channel B)
  if (gpt_pwmChannelB_enable) {
    infop->transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED;
    infop->p_dest = (void *)&(gpt_reg->GTCCR[GTCCR_E]);
    infop->transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED;
    infop->transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_EACH;
    infop->transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE; // unused
    infop->transfer_settings_word_b.size = TRANSFER_SIZE_4_BYTE;
    infop->transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL;
    infop->transfer_settings_word_b.irq = TRANSFER_IRQ_END;
    infop->num_blocks = 0; // unused
    infop++;
  }

  // GTIO pins / CHANNEL A & B
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

  // Stop Timer
  infop->transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->p_dest = (void *)&(gpt_reg->GTSTP);
  infop->transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->p_src = (void *)&(this->gpt_ChannelRegVal);
  infop->transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_EACH;
  infop->transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE; // unused
  infop->transfer_settings_word_b.size = TRANSFER_SIZE_4_BYTE;
  infop->transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL;
  infop->transfer_settings_word_b.irq = TRANSFER_IRQ_END;
  infop->num_blocks = 0; // unused.
  infop++;

  // Clear Timer
  infop->transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->p_dest = (void *)&(gpt_reg->GTCLR);
  infop->transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->p_src = (void *)&(this->gpt_ChannelRegVal);
  infop->transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_DISABLED;
  infop->transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE; // unused
  infop->transfer_settings_word_b.size = TRANSFER_SIZE_4_BYTE;
  infop->transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL;
  infop->transfer_settings_word_b.irq = TRANSFER_IRQ_END;
  infop->num_blocks = 0; // unused.
  infop++;

  dtc_dshot_info_reset(info);
}

void
DShotR4::dtc_dshot_info_reset(transfer_info_t *info)
{
  transfer_info_t *infop = info;
  uint16_t bits = waveformBits - 1;

  if (auto_restart) {
    bits += ifgBits;
  }

  // transfer waveform A
  if (gpt_pwmChannelA_enable) {
    infop->p_src = &(waveform[CHANNEL_A][1]);
    infop->length = bits;
    infop++;
  }

  // transfer waveform B
  if (gpt_pwmChannelB_enable) {
    infop->p_src = &(waveform[CHANNEL_B][1]);
    infop->length = bits;
    infop++;
  }

  // control GTIO pins.
  infop->p_src = &(gtioState[1]);
  infop->length = bits;
  infop++;

  // stop timer
  infop->length = 1;
  infop++;

  // clear timer
  infop->length = 1;
  infop++;
}

void
DShotR4::dtc_dshot_init(void)
{
  assert(GPT_IRQn != FSP_INVALID_VECTOR);

  dtc_dshot_info_init(dtc_dshot_info);
  dtc_cfg.p_info = dtc_dshot_info;

  dtc_extcfg.activation_source = GPT_IRQn;
  dtc_cfg.p_extend = &dtc_extcfg;

  fsp_err_t err = R_DTC_Open(&dtc_ctrl, &dtc_cfg);
  assert(FSP_SUCCESS == err);
}

bool
DShotR4::tx_restart()
{
  // can be called from interrupt context.
  assert(fsp_timer.is_opened() == true);
  assert(tx_busy == false);

  if (nextFrameUpdate) {
    load_dshot_frame();
  }

  dtc_dshot_info_reset(dtc_dshot_info);
  R_DTC_Reconfigure(&dtc_ctrl, dtc_dshot_info);
  R_DTC_Enable(&dtc_ctrl);

  if (gpt_pwmChannelA_enable) {
    fsp_timer.set_duty_cycle(waveform[0][0], CHANNEL_A);
  }
  if (gpt_pwmChannelB_enable) {
    fsp_timer.set_duty_cycle(waveform[1][0], CHANNEL_B);
  }
  gpt_reg->GTIOR = gtioState[0];

  tx_busy = true;
  fsp_timer.start();

  return true;
}

bool
DShotR4::tx_start()
{
  assert(fsp_timer.is_opened() == true);

  bool auto_restart_enabled = auto_restart;

  cancel_execution();

  auto_restart = auto_restart_enabled;
  nextFrameUpdate = true;

  return tx_restart();
}
