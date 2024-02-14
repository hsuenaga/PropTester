#include "DShotR4.h"

void
DShotR4::dshot_overflow_intr(timer_callback_args_t (*arg))
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
DShotR4::make_duty_counts(uint16_t frame)
{
  for (int i = 0; i < 16; i++) {
    bool bit = ((frame << i) & 0x8000) != 0;
    duty_table[i] = bit ? t1h_count : t0h_count;
  }
  duty_table[16] = stop_count;
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
  ext_cfg->gtior_setting.gtior_b.gtioa = 0x09;
  ext_cfg->gtior_setting.gtior_b.oadflt = 0;
  ext_cfg->gtior_setting.gtior_b.oahld = 0;
  ext_cfg->gtior_setting.gtior_b.oadf = 0;
  ext_cfg->gtior_setting.gtior_b.oae = 1;
}

void
DShotR4::gpt_gtioB_init(gpt_extended_cfg_t *ext_cfg)
{
  ext_cfg->gtior_setting.gtior_b.gtiob = 0x09;
  ext_cfg->gtior_setting.gtior_b.obdflt = 0;
  ext_cfg->gtior_setting.gtior_b.obhld = 0;
  ext_cfg->gtior_setting.gtior_b.obdf = 0;
  ext_cfg->gtior_setting.gtior_b.obe = 1;
}

void
DShotR4::gpt_init(void)
{
  // see variants/MINIMA/includes/ra/fsp/inc/api/r_ioport_api.h
  const static uint32_t pin_cfg = IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_GPT1;
  float default_duty = 0.0;
  float dshot_FreqHz = (float)dshot_timing[dshot_type].freqHz;

  // see cores/arduino/pinDefinitions.cpp
  pinPeripheral(gpt_pwmPin, pin_cfg);

  fsp_timer.begin(TIMER_MODE_PWM, GPT_TIMER, gpt_Channel, dshot_FreqHz, default_duty, dshot_overflow_intr, this);
  fsp_timer.setup_overflow_irq();
  fsp_timer.add_pwm_extended_cfg();
  gpt_extended_cfg_t *ext_cfg = (gpt_extended_cfg_t *)fsp_timer.get_cfg()->p_extend;

  switch (gpt_pwmChannel) {
    case CHANNEL_A:
      gpt_gtioA_init(ext_cfg);
      break;
    case CHANNEL_B:
      gpt_gtioB_init(ext_cfg);
      break;
    default:
      break;
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

  gpt_stop_cmd = 0x1 << gpt_Channel;
}

void
DShotR4::dtc_info_init(transfer_info_t *info)
{
  // transfer wave forms
  info[0].transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  info[0].transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED;
  info[0].transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_END;
  info[0].transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE; // unused
  info[0].transfer_settings_word_b.size = TRANSFER_SIZE_4_BYTE;
  info[0].transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL;
  info[0].transfer_settings_word_b.irq = TRANSFER_IRQ_END;

  // stop GTP
  info[1].transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  info[1].transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  info[1].transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_DISABLED;
  info[1].transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE; // unused
  info[1].transfer_settings_word_b.size = TRANSFER_SIZE_4_BYTE;
  info[1].transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL;
  info[1].transfer_settings_word_b.irq = TRANSFER_IRQ_END;

  dtc_info_reset(info);
}

void
DShotR4::dtc_info_reset(transfer_info_t *info)
{
  void *bufferReg;
  void *stopReg;
  enum GTCCR_Map bufferRegOff;

  switch (gpt_pwmChannel) {
    case CHANNEL_A:
      bufferRegOff = GTCCR_C;
      break;
    case CHANNEL_B:
      bufferRegOff = GTCCR_E;
      break;
    default:
      return;
  }
  
  switch (gpt_Channel) {
    case 0:
      bufferReg = (void *)&(R_GPT0->GTCCR[bufferRegOff]);
      stopReg = (void *)&(R_GPT0->GTSTP);
      break;
    case 1:
      bufferReg = (void *)&(R_GPT1->GTCCR[bufferRegOff]);
      stopReg = (void *)&(R_GPT1->GTSTP);
      break;
    case 2:
      bufferReg = (void *)&(R_GPT2->GTCCR[bufferRegOff]);
      stopReg = (void *)&(R_GPT2->GTSTP);
      break;
    case 3:
      bufferReg = (void *)&(R_GPT3->GTCCR[bufferRegOff]);
      stopReg = (void *)&(R_GPT3->GTSTP);
      break;
    case 4:
      bufferReg = (void *)&(R_GPT4->GTCCR[bufferRegOff]);
      stopReg = (void *)&(R_GPT4->GTSTP);
      break;
    default:
      return;
  }

  // transfer wave forms
  info[0].p_src = &(duty_table[1]);
  info[0].p_dest = bufferReg;
  info[0].num_blocks = 0; // unused
  info[0].length = sizeof(duty_table) / sizeof(duty_table[0]);

  // stop GTP
  info[1].p_src = &gpt_stop_cmd;
  info[1].p_dest = stopReg;
  info[1].num_blocks = 0; // unused.
  info[1].length = 1;
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
DShotR4::tx_start(uint16_t frame)
{
  assert(fsp_timer.is_opened() == true);

  if (tx_busy) {
    tx_error++;
    return false;
  }
  tx_busy = true;

  make_duty_counts(frame);

  fsp_timer.stop();
  fsp_timer.reset();

  fsp_timer.set_duty_cycle(duty_table[0], gpt_pwmChannel);
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
DShotR4::init(enum DShotType type, uint8_t channel, TimerPWMChannel_t pwmChannel, pin_size_t pwmPin)
{
  this->dshot_type = type;
  this->gpt_Channel = channel;
  this->gpt_pwmChannel = pwmChannel;
  this->gpt_pwmPin = pwmPin;
  gpt_init();
  dtc_init();
}

bool
DShotR4::send_rawValue(uint16_t value, bool telemetry)
{
  uint16_t data, crc, frame;

  data = (value << 1) | (telemetry ? 0x0001 : 0x0000);
  crc = (data ^ (data >> 4) ^ (data >> 8)) &0x000f;
  frame = (data << 4) | crc;

  return tx_start(frame);
}
bool
DShotR4::send_throttle(uint16_t throttle, bool telemetry)
{
  uint16_t value, crc, frame;

  if (throttle < 48 || throttle > 0x07ff) {
    return false;
  }
  return send_rawValue(throttle, telemetry);
}

bool
DShotR4::send_command(enum DShotCommand cmd)
{
  return send_rawValue((uint16_t)cmd, false);
}

bool
DShotR4::send_testPattern()
{
  return send_rawValue(0x0555, false);
}