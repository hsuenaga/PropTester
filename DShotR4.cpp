#include "DShotR4.h"

#if defined(ARDUINO_MINIMA) || defined(ARDUINO_UNOWIFIR4)
#else
#error "Board not supported."
#endif

void
DShotR4::gpt_overflow_intr(timer_callback_args_t (*arg))
{
  DShotR4 *obj = (DShotR4 *)arg->p_context;

  obj->tx_complete(arg);
}

void
DShotR4::tx_complete(timer_callback_args_t (*arg))
{
  tx_success++;
  tx_busy = false;
  fsp_timer.stop();
  if (auto_restart) {
    tx_restart();
  }

  return;
}

void
DShotR4::load_frame()
{
  // NEVER call while DTC is running.
  assert(tx_busy == false);

  for (int i = 0; i < 16; i++) {    
    bool bit;

    if (gpt_pwmChannelA_enable) {
      bit = ((nextFrame[0] << i) & 0x8000) != 0;
      waveform[0][i] = bit ? t1h_count : t0h_count;
    }
    if (gpt_pwmChannelB_enable) {
      bit = ((nextFrame[1] << i) & 0x8000) != 0;
      waveform[1][i] = bit ? t1h_count : t0h_count;
    }
  }

  nextFrameUpdate = false;
}

void
DShotR4::timing_init(void)
{
  assert(fsp_timer.is_opened() == true);

  const struct dshot_params_t *param = &dshot_timing[dshot_type];

  period_count = fsp_timer.get_cfg()->period_counts;

  float t1h_ratio = param->t1h_us / param->bit_duration_us;
  float t0h_ratio = param->t0h_us / param->bit_duration_us;

  t1h_count = (uint32_t)ceil((float)period_count * t1h_ratio);
  t0h_count = (uint32_t)ceil((float)period_count * t0h_ratio);
  stop_count = 0;
  dshot_ifg_us = (uint32_t)ceil(param->bit_duration_us * ifgBits);
}

void
DShotR4::gpt_gtioState_init()
{
  gtioRunning.gtior = 0;
  gtioStop.gtior = 0;
  gtioReset.gtior = 0;

  if (gpt_pwmChannelA_enable) {
    if (dshotInvertA) {
      gtioRunning.gtior_b.gtioa = 0x16; // High, Low, High
      gtioRunning.gtior_b.oadflt = 1; // High
      gtioStop.gtior_b.gtioa = 0x1A; // High, High, High
      gtioStop.gtior_b.oadflt = 1; // High
      gtioReset.gtior_b.gtioa = 0x05; // Low, Low, Low
      gtioReset.gtior_b.oadflt = 0; // Low
    }
    else {
      gtioRunning.gtior_b.gtioa = 0x09; // Low, High, Low
      gtioRunning.gtior_b.oadflt = 0; // Low
      gtioStop.gtior_b.gtioa = 0x05; // Low, Low, Low
      gtioStop.gtior_b.oadflt = 0; // Low
      gtioReset.gtior_b.gtioa = 0x1A; // High, High, High
      gtioReset.gtior_b.oadflt = 1; // High
    }
    gtioRunning.gtior_b.oae = 1;
    gtioStop.gtior_b.oae = 1;
    gtioReset.gtior_b.oae = 1;
  }

  if (gpt_pwmChannelB_enable) {
    if (dshotInvertB) {
      gtioRunning.gtior_b.gtiob = 0x16; // High, Low, High
      gtioRunning.gtior_b.obdflt = 1; // High
      gtioStop.gtior_b.gtiob = 0x1A; // High, High, High
      gtioStop.gtior_b.obdflt = 1; // High
      gtioReset.gtior_b.gtiob = 0x05; // Low, Low, Low
      gtioReset.gtior_b.obdflt = 0; // Low
    }
    else {
      gtioRunning.gtior_b.gtiob = 0x09; // Low, High, Low
      gtioRunning.gtior_b.obdflt = 0; // Low
      gtioStop.gtior_b.gtiob = 0x05; // Low, Low, Low
      gtioStop.gtior_b.obdflt = 0; // Low
      gtioReset.gtior_b.gtiob = 0x1A; // High, High, High
      gtioReset.gtior_b.obdflt = 1; // High
    }
    gtioRunning.gtior_b.obe = 1;
    gtioStop.gtior_b.obe = 1;
    gtioReset.gtior_b.obe = 1;
  }

  for (int i = 0; i < (waveformBits - 1); i++) {
    gtioState[i] = gtioRunning.gtior;
  }
  for (int i = (waveformBits - 1); i < (waveformBits + ifgBits); i++) {
    gtioState[i] = gtioStop.gtior;
  }
}

void
DShotR4::gpt_init(void)
{
  // see variants/MINIMA/includes/ra/fsp/inc/api/r_ioport_api.h
  const static uint32_t pin_cfg = IOPORT_CFG_PORT_DIRECTION_OUTPUT |
                                  IOPORT_CFG_DRIVE_MID |
                                  IOPORT_CFG_PERIPHERAL_PIN |
                                  IOPORT_PERIPHERAL_GPT1;
  float default_duty = 0.0;
  float freqHz = (float)(1.0 / dshot_timing[dshot_type].bit_duration_us * 1000.0 * 1000.0);

  gpt_reg = (R_GPT0_Type *)((unsigned long)R_GPT0_BASE + (unsigned long)(0x0100 * gpt_Channel));
  gpt_gtioState_init();

  fsp_timer.begin(TIMER_MODE_PWM, GPT_TIMER, gpt_Channel, freqHz, default_duty, gpt_overflow_intr, this);
  fsp_timer.setup_overflow_irq();
  fsp_timer.add_pwm_extended_cfg();
  gpt_extended_cfg_t *ext_cfg = (gpt_extended_cfg_t *)fsp_timer.get_cfg()->p_extend;
  ext_cfg->gtior_setting.gtior = gtioStop.gtior;

  if (gpt_pwmChannelA_enable) {
      pinPeripheral(gpt_pwmPinA, pin_cfg);
  }
  if (gpt_pwmChannelB_enable) {
      pinPeripheral(gpt_pwmPinB, pin_cfg);
  }
  fsp_timer.open();

  // Get period count from current config.
  timing_init();

  // Get IRQn from current config.
  for (int i = 0; i < sizeof(R_ICU->IELSR)/sizeof(R_ICU->IELSR[0]); i++) {
    if (R_ICU->IELSR[i] == BSP_PRV_IELS_ENUM(EVENT_GPT4_COUNTER_OVERFLOW)) {
      GPT_IRQn = (IRQn)i;
    }
  }

  // Initialize waveforms
  for (int i = 0; i < (waveformBits + ifgBits); i++) {
    waveform[0][i] = waveform[1][i] = stop_count;
  }
}

void
DShotR4::dtc_info_init(transfer_info_t *info)
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

  // GTIO pins
  infop->transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->p_dest = (void *)&(gpt_reg->GTIOR);
  infop->transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED;
  infop->transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_DISABLED;
  infop->transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE; // unused
  infop->transfer_settings_word_b.size = TRANSFER_SIZE_4_BYTE;
  infop->transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL;
  infop->transfer_settings_word_b.irq = TRANSFER_IRQ_END;
  infop->num_blocks = 0; // unused.

  dtc_info_reset(info);
}

void
DShotR4::dtc_info_reset(transfer_info_t *info)
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

  // disable GTIO pins.
  infop->p_src = &(gtioState[1]);
  infop->length = bits;
}

void
DShotR4::dtc_init(void)
{
  assert(GPT_IRQn != FSP_INVALID_VECTOR);

  dtc_info_init(dtc_info);
  dtc_cfg.p_info = dtc_info;

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
    load_frame();
  }

  if (gpt_pwmChannelA_enable) {
    fsp_timer.set_duty_cycle(waveform[0][0], CHANNEL_A);
  }
  if (gpt_pwmChannelB_enable) {
    fsp_timer.set_duty_cycle(waveform[1][0], CHANNEL_B);
  }
  gpt_reg->GTIOR = gtioState[0];

  dtc_info_reset(dtc_info);
  R_DTC_Reconfigure(&dtc_ctrl, dtc_info);
  R_DTC_Enable(&dtc_ctrl);
  tx_busy = true;

  fsp_timer.reset();
  fsp_timer.start();

  return true;
}

bool
DShotR4::tx_start()
{
  assert(fsp_timer.is_opened() == true);

  bool auto_restart_enabled = auto_restart;
  
  auto_restart = false;
  while (tx_busy) {
    yield();
  }

  fsp_timer.stop();
  fsp_timer.reset();
  R_DTC_Disable(&dtc_ctrl);
  auto_restart = auto_restart_enabled;
  nextFrameUpdate = true;

  return tx_restart();
}

/*
 * public
 */
DShotR4::DShotR4(): tx_success(0), fsp_timer()
{
}

DShotR4::~DShotR4()
{  
  deinit();
}

bool
DShotR4::init(enum DShotType type, bool biDir, uint8_t channel, bool useChannelA, bool useChannelB, pin_size_t pwmPinA, pin_size_t pwmPinB)
{
  this->dshot_type = type;
  this->gpt_Channel = channel;
  this->gpt_pwmChannelA_enable = useChannelA;
  this->gpt_pwmChannelB_enable = useChannelB;
  this->gpt_pwmPinA = pwmPinA;
  this->gpt_pwmPinB = pwmPinB;
  if (biDir && type != DSHOT150) {
    this->dshotInvertA = true;
    this->dshotInvertB = true;
  }
  this->auto_restart = false;

  gpt_init();
  dtc_init();
}

bool
DShotR4::deinit()
{
  transfer_properties_t dtc_prop;

  auto_restart = false;
  while(tx_busy) {
    yield();
  }

  if (R_DTC_InfoGet(&dtc_ctrl, &dtc_prop) == FSP_SUCCESS) {
    R_DTC_Disable(&dtc_ctrl);
    R_DTC_Close(&dtc_ctrl);
    memset(&dtc_ctrl, 0, sizeof(dtc_ctrl));
  }
  
  if (fsp_timer.is_opened()) {
    fsp_timer.stop();
    fsp_timer.close();
  }
}

bool
DShotR4::set_rawValue(TimerPWMChannel_t channel, uint16_t value, bool telemetry)
{
  uint16_t data, cksum, frame;

  if (channel > CHANNEL_B) {
    return false;
  }
  data = (value << 1) | (telemetry ? 0x0001 : 0x0000);
  cksum = (data ^ (data >> 4) ^ (data >> 8)) & 0x000f;
  if ((channel == CHANNEL_A && dshotInvertA) || (channel == CHANNEL_B && dshotInvertB)) {
    frame = (data << 4) | ~cksum;
  }
  else {
    frame = (data << 4) | cksum;
  }

  nextFrame[channel] = frame;
  return true;;
}

bool
DShotR4::set_throttle(TimerPWMChannel_t channel, uint16_t throttle, bool telemetry)
{
  uint16_t value, crc, frame;

  if (throttle < 48 || throttle > 0x07ff) {
    return false;
  }
  return set_rawValue(channel, throttle, telemetry);
}

bool
DShotR4::set_command(TimerPWMChannel_t channel, enum DShotCommand cmd)
{
  return set_rawValue(channel, (uint16_t)cmd, true);
}

bool
DShotR4::set_testPattern()
{
  set_rawValue(CHANNEL_A, 0x0555, true);
  set_rawValue(CHANNEL_B, 0x0AAA, false);
  return true;
}

bool
DShotR4::transmit(bool oneShot)
{
  if (fsp_timer.is_opened() == false) {
    return false;
  }
  if ((!gpt_pwmChannelA_enable) && (!gpt_pwmChannelB_enable)) {
    return false;
  }

  if (auto_restart && oneShot == false) {
    nextFrameUpdate = true;
    return true;
  }

  if (oneShot) {
    auto_restart = false;
    tx_start();
    while(tx_busy) {
      yield();
    }
    return true;
  }

  auto_restart = true;
  return tx_start();
}

bool
DShotR4::suspend()
{
  if (fsp_timer.is_opened() == false) {
    return false;
  }
  if ((!gpt_pwmChannelA_enable) && (!gpt_pwmChannelB_enable)) {
    return false;
  }
  auto_restart = false;
  while (tx_busy) {
    yield();
  }
  armed = false;

  return true;
}

bool
DShotR4::arm()
{
  if (fsp_timer.is_opened() == false) {
    return false;
  }
  if ((!gpt_pwmChannelA_enable) && (!gpt_pwmChannelB_enable)) {
    return false;
  }

  // send disarm
  set_command(CHANNEL_A, DSHOT_CMD_DISARM);
  set_command(CHANNEL_B, DSHOT_CMD_DISARM);
  transmit();
  delay(300);

  // send 0%
  set_throttle(CHANNEL_A, 48);
  set_throttle(CHANNEL_B, 48);
  transmit();

  armed = true;

  return true;
}

bool
DShotR4::reset()
{
  gpt_reg->GTIOR = gtioReset.gtior;
  delay(1000);
  gpt_reg->GTIOR = gtioStop.gtior;
  delay(1000);
}

uint32_t
DShotR4::get_ifg_us()
{
  return dshot_ifg_us;
}
