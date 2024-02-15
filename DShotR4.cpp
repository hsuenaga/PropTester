#include "DShotR4.h"

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

  return;
}

void
DShotR4::make_duty_counts(uint32_t table[], uint16_t frame)
{
  for (int i = 0; i < 16; i++) {
    bool bit = ((frame << i) & 0x8000) != 0;
    table[i] = bit ? t1h_count : t0h_count;
  }
  table[16] = stop_count;
}

void
DShotR4::timing_init(void)
{
  assert(fsp_timer.is_opened() == true);

  period_count = fsp_timer.get_cfg()->period_counts;

  float dshot_T1H_Duty = (dshot_timing[dshot_type].t1h_us / dshot_timing[dshot_type].bit_duration_us);
  float dshot_T0H_Duty = (dshot_timing[dshot_type].t0h_us / dshot_timing[dshot_type].bit_duration_us);
  t1h_count = (uint32_t)ceil((float)period_count * dshot_T1H_Duty);
  t0h_count = (uint32_t)ceil((float)period_count * dshot_T0H_Duty);
  stop_count = 0;
}

void
DShotR4::gpt_gtioA_init(gpt_extended_cfg_t *ext_cfg)
{
  if (dshotInvertA) {
    ext_cfg->gtior_setting.gtior_b.gtioa = 0x16;
    ext_cfg->gtior_setting.gtior_b.oadflt = 1;
    ext_cfg->gtior_setting.gtior_b.oahld = 0;
    ext_cfg->gtior_setting.gtior_b.oadf = 0;
    ext_cfg->gtior_setting.gtior_b.oae = 1;
  }
  else {
    ext_cfg->gtior_setting.gtior_b.gtioa = 0x09;
    ext_cfg->gtior_setting.gtior_b.oadflt = 0;
    ext_cfg->gtior_setting.gtior_b.oahld = 0;
    ext_cfg->gtior_setting.gtior_b.oadf = 0;
    ext_cfg->gtior_setting.gtior_b.oae = 1;
  }
}

void
DShotR4::gpt_gtioB_init(gpt_extended_cfg_t *ext_cfg)
{
  if (dshotInvertB) {
    ext_cfg->gtior_setting.gtior_b.gtiob = 0x16;
    ext_cfg->gtior_setting.gtior_b.obdflt = 1;
    ext_cfg->gtior_setting.gtior_b.obhld = 0;
    ext_cfg->gtior_setting.gtior_b.obdf = 0;
    ext_cfg->gtior_setting.gtior_b.obe = 1;
  }
  else {
    ext_cfg->gtior_setting.gtior_b.gtiob = 0x09;
    ext_cfg->gtior_setting.gtior_b.obdflt = 0;
    ext_cfg->gtior_setting.gtior_b.obhld = 0;
    ext_cfg->gtior_setting.gtior_b.obdf = 0;
    ext_cfg->gtior_setting.gtior_b.obe = 1;
  }
}

void
DShotR4::gpt_init(void)
{
  // see variants/MINIMA/includes/ra/fsp/inc/api/r_ioport_api.h
  const static uint32_t pin_cfg = IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_GPT1;
  float default_duty = 0.0;
  float dshot_FreqHz = (float)dshot_timing[dshot_type].freqHz;

  // see cores/arduino/pinDefinitions.cpp
  if (gpt_pwmChannelA_enable) {
    pinPeripheral(gpt_pwmPinA, pin_cfg);
  }
  if (gpt_pwmChannelB_enable) {
    pinPeripheral(gpt_pwmPinB, pin_cfg);
  }

  fsp_timer.begin(TIMER_MODE_PWM, GPT_TIMER, gpt_Channel, dshot_FreqHz, default_duty, gpt_overflow_intr, this);
  fsp_timer.setup_overflow_irq();
  fsp_timer.add_pwm_extended_cfg();
  gpt_extended_cfg_t *ext_cfg = (gpt_extended_cfg_t *)fsp_timer.get_cfg()->p_extend;

  if (gpt_pwmChannelA_enable) {
      gpt_gtioA_init(ext_cfg);
  }
  if (gpt_pwmChannelB_enable) {
      gpt_gtioB_init(ext_cfg);
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

  gpt_reg = (R_GPT0_Type *)((unsigned long)R_GPT0_BASE + (unsigned long)(0x0100 * gpt_Channel));
}

void
DShotR4::dtc_info_init(transfer_info_t *info)
{
  transfer_info_t *infop;

  // transfer wave forms (channel A)
  infop = info;
  infop->transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED;
  infop->transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_EACH;
  infop->transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE; // unused
  infop->transfer_settings_word_b.size = TRANSFER_SIZE_4_BYTE;
  infop->transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL;
  infop->transfer_settings_word_b.irq = TRANSFER_IRQ_END;

  // transfer wave forms (channel B)
  infop++;
  infop->transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED;
  infop->transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_END;
  infop->transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE; // unused
  infop->transfer_settings_word_b.size = TRANSFER_SIZE_4_BYTE;
  infop->transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL;
  infop->transfer_settings_word_b.irq = TRANSFER_IRQ_END;

  // stop GTP
  infop++;
  infop->transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  infop->transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_DISABLED;
  infop->transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE; // unused
  infop->transfer_settings_word_b.size = TRANSFER_SIZE_4_BYTE;
  infop->transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL;
  infop->transfer_settings_word_b.irq = TRANSFER_IRQ_END;

  dtc_info_reset(info);
}

void
DShotR4::dtc_info_reset(transfer_info_t *info)
{
  transfer_info_t *infop = info;
  static uint32_t gpt_stop_cmd;

  gpt_stop_cmd = 0x1 << gpt_Channel;

  // transfer wave forms A
  infop = info;
  infop->p_src = &(duty_table[CHANNEL_A][1]);
  infop->p_dest = (void *)&(gpt_reg->GTCCR[GTCCR_C]);
  infop->num_blocks = 0; // unused
  infop->length = sizeof(duty_table[0]) / sizeof(duty_table[0][0]);

  // transfer wave forms B
  infop++;
  infop->p_src = &(duty_table[CHANNEL_B][1]);
  infop->p_dest = (void *)&(gpt_reg->GTCCR[GTCCR_E]);
  infop->num_blocks = 0; // unused
  infop->length = sizeof(duty_table[0]) / sizeof(duty_table[0][0]);

  // stop GTP
  infop++;
  infop->p_src = &gpt_stop_cmd;
  infop->p_dest = (void *)&(gpt_reg->GTSTP);
  infop->num_blocks = 0; // unused.
  infop->length = 1;
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

  err = R_DTC_Enable(&dtc_ctrl);
  assert(FSP_SUCCESS == err);
}

bool
DShotR4::tx_start()
{
  assert(fsp_timer.is_opened() == true);

  if (tx_busy) {
    tx_error++;
    return false;
  }
  tx_busy = true;

  fsp_timer.stop();
  fsp_timer.reset();
  for (int i = CHANNEL_A; i <= CHANNEL_B; i++) {
    make_duty_counts(duty_table[i], dshotFrame[i]);
    fsp_timer.set_duty_cycle(duty_table[i][0], (TimerPWMChannel_t)i);
  }
  dtc_info_reset(dtc_info);
  R_DTC_Reconfigure(&dtc_ctrl, dtc_info);

  fsp_timer.start();

  return true;
}

/*
 * public
 */
DShotR4::DShotR4(): tx_success(0), fsp_timer()
{
}

DShotR4::~DShotR4()
{
  if (fsp_timer.is_opened()) {
    fsp_timer.stop();
    fsp_timer.close();
  }
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

  gpt_init();
  dtc_init();
}

bool
DShotR4::set_rawValue(TimerPWMChannel_t channel, uint16_t value, bool telemetry)
{
  uint16_t data, crc, frame;

  if (channel > CHANNEL_B) {
    return false;
  }
  data = (value << 1) | (telemetry ? 0x0001 : 0x0000);
  crc = (data ^ (data >> 4) ^ (data >> 8)) &0x000f;
  frame = (data << 4) | crc;

  dshotFrame[channel] = frame;
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
  return set_rawValue(channel, (uint16_t)cmd, false);
}

bool
DShotR4::set_testPattern()
{
  set_rawValue(CHANNEL_A, 0x0555, false);
  set_rawValue(CHANNEL_B, 0x0AAA, false);
  return true;
}

bool
DShotR4::transmit()
{
  if (fsp_timer.is_opened() == false) {
    Serial.println("fsp_timer is not initialized.");
    return false;
  }
  if ((!gpt_pwmChannelA_enable) && (!gpt_pwmChannelB_enable)) {
    Serial.println("All pwm channels disabled.");
    return false;
  }
  return tx_start();
}
