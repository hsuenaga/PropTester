#include "DShotR4.h"


void
DShotR4::dshot_intr0(timer_callback_args_t (*arg))
{
  DShotR4 *obj = (DShotR4 *)arg->p_context;

  obj->dshot_intr(arg);
}

void
DShotR4::dshot_intr(timer_callback_args_t (*arg))
{
  intr_count++;
  running = false;
  if (enable_dtc || *cur_duty == stop_count) {
    fsp_timer.stop();
    return;
  }
  cur_duty++;
 #if 1
  R_GPT4->GTCCR[GTCCR_E] = *cur_duty;
#else
  // too slow.
  fsp_timer.set_duty_cycle(*cur_duty, CHANNEL_B);
#endif
}

void
DShotR4::dshot_setup_duty_table(uint16_t frame)
{
  for (int i = 0; i < 16; i++) {
    bool bit = ((frame << i) & 0x8000) != 0;
    duty_table[i] = bit ? t1h_count : t0h_count;
  }
  duty_table[16] = stop_count;
  cur_duty = &duty_table[0];
}

void
DShotR4::dshot_param_init(void)
{
  period_count = fsp_timer.get_cfg()->period_counts;

  float dshot_T1H_Duty = (dshot_timing[dshot_type].t1h_us / dshot_timing[dshot_type].bit_duration_us);
  float dshot_T0H_Duty = (dshot_timing[dshot_type].t0h_us / dshot_timing[dshot_type].bit_duration_us);
  t1h_count = (uint32_t)((float)period_count * dshot_T1H_Duty);
  t0h_count = (uint32_t)((float)period_count * dshot_T0H_Duty);
  stop_count = 0;
}

void
DShotR4::dshot_pin_init(void)
{
  pinMode(pin_dshot, OUTPUT);
  pinMode(pin_dshot_ref, OUTPUT);
}

void
DShotR4::dshot_gpt_init(void)
{
  // see variants/MINIMA/includes/ra/fsp/inc/api/r_ioport_api.h
  uint32_t pin_cfg = IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_GPT1;

  // see cores/arduino/pinDefinitions.cpp
  pinPeripheral(pin_dshot, pin_cfg);
  pinPeripheral(pin_dshot_ref, pin_cfg);

  uint8_t type = GPT_TIMER;
  uint8_t channel = 4;
  float default_duty = 0.0;
  float dshot_FreqHz = (1.0F/(dshot_timing[dshot_type].bit_duration_us/1000000.0F));

  fsp_timer.begin(TIMER_MODE_PWM, type, channel, dshot_FreqHz, default_duty, dshot_intr0, this);
  fsp_timer.setup_overflow_irq();

  fsp_timer.add_pwm_extended_cfg();

  gpt_extended_cfg_t *ext_cfg = (gpt_extended_cfg_t *)fsp_timer.get_cfg()->p_extend;
  
  // Channel A (D1, P302, GTIOC4A), not used
  ext_cfg->gtior_setting.gtior_b.gtioa = 0x09;
  ext_cfg->gtior_setting.gtior_b.oadflt = 0;
  ext_cfg->gtior_setting.gtior_b.oahld = 0;
  ext_cfg->gtior_setting.gtior_b.oadf = 0;
  ext_cfg->gtior_setting.gtior_b.oae = 1;

  // Channel B (D0, P301, GTIOC4B)
  ext_cfg->gtior_setting.gtior_b.gtiob = 0x09;
  ext_cfg->gtior_setting.gtior_b.obdflt = 0;
  ext_cfg->gtior_setting.gtior_b.obhld = 0;
  ext_cfg->gtior_setting.gtior_b.obdf = 0;
  ext_cfg->gtior_setting.gtior_b.obe = 1;

  fsp_timer.open();

  dshot_param_init();
  fsp_timer.set_duty_cycle(period_count/2, CHANNEL_A);

  // Get IRQn from current config.
  for (int i = 0; i < sizeof(R_ICU->IELSR)/sizeof(R_ICU->IELSR[0]); i++) {
    if (R_ICU->IELSR[i] == BSP_PRV_IELS_ENUM(EVENT_GPT4_COUNTER_OVERFLOW)) {
      GPT_IRQn = (IRQn)i;
    }
  }

  gpt_stop_cmd = 0x1 << channel;
}

void
DShotR4::dshot_dtc_init(void)
{
  if (!enable_dtc) {
    Serial.println("DTC Disabled.");
    return;
  }

  dtc_info_template[0].transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  dtc_info_template[0].transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED;
  dtc_info_template[0].transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_END;
  dtc_info_template[0].transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE; // unused
  dtc_info_template[0].transfer_settings_word_b.size = TRANSFER_SIZE_4_BYTE;
  dtc_info_template[0].transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL;
  dtc_info_template[0].transfer_settings_word_b.irq = TRANSFER_IRQ_END;
  // following data will be written back by DTC.
  dtc_info_template[0].p_src = &(duty_table[1]);
  dtc_info_template[0].p_dest = (void *)&(R_GPT4->GTCCR[GTCCR_E]);
  dtc_info_template[0].num_blocks = 0; // unused
  dtc_info_template[0].length = sizeof(duty_table) / sizeof(duty_table[0]);

  dtc_info_template[1].transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  dtc_info_template[1].transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_FIXED;
  dtc_info_template[1].transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_DISABLED;
  dtc_info_template[1].transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE; // unused
  dtc_info_template[1].transfer_settings_word_b.size = TRANSFER_SIZE_4_BYTE;
  dtc_info_template[1].transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL;
  dtc_info_template[1].transfer_settings_word_b.irq = TRANSFER_IRQ_END;

  dtc_info_template[1].p_src = &gpt_stop_cmd;
  dtc_info_template[1].p_dest = (void *)(&R_GPT4->GTSTP);
  dtc_info_template[1].num_blocks = 0; // unused.
  dtc_info_template[1].length = 1;

  assert(GPT_IRQn != FSP_INVALID_VECTOR);
  dtc_extcfg.activation_source = GPT_IRQn;

  memcpy(dtc_info, dtc_info_template, sizeof(dtc_info));
  dtc_cfg.p_info = dtc_info;
  dtc_cfg.p_extend = &dtc_extcfg;

  fsp_err_t err = R_DTC_Open(&dtc_ctrl, &dtc_cfg);
  assert(FSP_SUCCESS == err);

  err = R_DTC_Enable(&dtc_ctrl);
  assert(FSP_SUCCESS == err);

  Serial.println("DTC Enabled.");
}

DShotR4::DShotR4(): intr_count(0), dshot_port(0), fsp_timer()
{
}

DShotR4::~DShotR4()
{
  fsp_timer.stop();
  fsp_timer.close();
}

bool
DShotR4::init(enum DShotType type, bool dtc_flag)
{
  dshot_type = type;
  enable_dtc = dtc_flag;
  dshot_pin_init();
  dshot_gpt_init();
  dshot_dtc_init();
}

bool
DShotR4::start(uint16_t frame)
{
  if (running)
    return false;

  dshot_setup_duty_table(frame);
  Serial.println(intr_count);

  if (fsp_timer.is_opened()) {
    fsp_timer.set_duty_cycle(*cur_duty, CHANNEL_B);
    fsp_timer.stop();
    fsp_timer.reset();
    if (enable_dtc) {
      memcpy(dtc_info, dtc_info_template, sizeof(dtc_info));   
      R_DTC_Reconfigure(&dtc_ctrl, dtc_info);
    }
    fsp_timer.start();

    running = true;
    return true;
  }

  return false;
}

bool
DShotR4::stop(void)
{
  if (fsp_timer.is_opened()) {
    return true;
    fsp_timer.stop();
  }

  return false;
}


bool
DShotR4::send_rawValue(uint16_t value, bool telemetry)
{
  uint16_t data, crc, frame;

  data = (value << 1) | (telemetry ? 0x0001 : 0x0000);
  crc = (data ^ (data >> 4) ^ (data >> 8)) &0x000f;
  frame = (data << 4) | crc;

  return start(frame);
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
