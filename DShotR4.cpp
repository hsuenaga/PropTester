#include "DShotR4.h"

void
DShotR4::gpt_overflow_intr(timer_callback_args_t (*arg))
{
  DShotR4 *instance = (DShotR4 *)arg->p_context;

  if (instance->serialCore.is_tx_busy() || instance->serialCore.is_rx_ready()) {
    return instance->serialCore.overflow_interrupt(arg);
  }
  else {
    return instance->tx_dshot_complete(arg);
  }

  instance->spurious_intr++;
}

void
DShotR4::gpt_gtioState_init()
{
  uint32_t oae = gpt_pwmChannelA_enable ? 1 : 0;
  uint32_t obe = gpt_pwmChannelB_enable ? 1 : 0;

  // Fixed level
  gtioHigh.gtior = 0;
  gtioLow.gtior = 0;
  gtioHigh.gtior_b.oae = gtioLow.gtior_b.oae = oae;
  gtioHigh.gtior_b.obe = gtioLow.gtior_b.obe = obe;
  gtioHigh.gtior_b.gtioa = gtioHigh.gtior_b.gtiob = 0x1A; // High, High, High
  gtioHigh.gtior_b.oadflt = gtioHigh.gtior_b.obdflt = 1; // High
  gtioLow.gtior_b.gtioa = gtioLow.gtior_b.gtiob = 0x05; // Low, Low, Low
  gtioLow.gtior_b.oadflt = gtioLow.gtior_b.obdflt = 0; // Low

  // DShot wave forms
  gtioRunning.gtior = 0;
  gtioStop.gtior = 0;

  gtioRunning.gtior_b.oae = oae;
  gtioStop.gtior_b.oae = oae;
  if (dshotInvertA) {
    gtioRunning.gtior_b.gtioa = 0x16; // High, Low, High
    gtioRunning.gtior_b.oadflt = 1; // High
    gtioStop.gtior_b.gtioa = gtioHigh.gtior_b.gtioa;
    gtioStop.gtior_b.oadflt = gtioHigh.gtior_b.oadflt;
  }
  else {
    gtioRunning.gtior_b.gtioa = 0x09; // Low, High, Low
    gtioRunning.gtior_b.oadflt = 0; // Low
    gtioStop.gtior_b.gtioa = gtioLow.gtior_b.gtioa;
    gtioStop.gtior_b.oadflt = gtioLow.gtior_b.oadflt;
  }

  gtioRunning.gtior_b.obe = obe;
  gtioStop.gtior_b.obe = obe;
  if (dshotInvertB) {
    gtioRunning.gtior_b.gtiob = 0x16; // High, Low, High
    gtioRunning.gtior_b.obdflt = 1; // High
    gtioStop.gtior_b.gtiob = gtioHigh.gtior_b.gtiob;
    gtioStop.gtior_b.obdflt = gtioHigh.gtior_b.obdflt;
  }
  else {
    gtioRunning.gtior_b.gtiob = 0x09; // Low, High, Low
    gtioRunning.gtior_b.obdflt = 0; // Low
    gtioStop.gtior_b.gtiob = gtioLow.gtior_b.gtiob;
    gtioStop.gtior_b.obdflt = gtioLow.gtior_b.obdflt;
  }
#ifdef DEBUG_CHANNEL_A
  gtioStop.gtior_b.gtioa = gtioRunning.gtior_b.gtioa;
  gtioStop.gtior_b.oadflt = gtioRunning.gtior_b.oadflt;
#endif

  for (int i = 0; i < (waveformBits - 1); i++) {
    gtioState[i] = gtioRunning.gtior;
  }
  for (int i = (waveformBits - 1); i < dshotBits; i++) {
    gtioState[i] = gtioStop.gtior;
  }
}

void
DShotR4::gpt_init(void)
{
  float default_duty = 0.0;
  float default_freqHz = 1.0;

  gpt_reg = (R_GPT0_Type *)((unsigned long)R_GPT0_BASE + (unsigned long)(0x0100 * gpt_Channel));
  gpt_ChannelRegVal = 0x1 << gpt_Channel;
  gpt_gtioState_init();

  // reserve GPT via FspTimer class to avoid conflict with Arduino-Core functions.
  fsp_timer.begin(TIMER_MODE_PWM, GPT_TIMER, gpt_Channel, default_freqHz, default_duty, gpt_overflow_intr, this);
  fsp_timer.setup_overflow_irq();
  fsp_timer.set_source_start(GPT_SOURCE_GPT_A); // XXX: restricted event source...
  fsp_timer.add_pwm_extended_cfg();
  ((gpt_extended_cfg_t *)fsp_timer.get_cfg()->p_extend)->gtior_setting.gtior = gtioStop.gtior;
  fsp_timer.open();

  // Get IRQn from current environment. need to update if fsp_timer.end() is called.
  for (int i = 0; i < sizeof(R_ICU->IELSR)/sizeof(R_ICU->IELSR[0]); i++) {
    if (R_ICU->IELSR[i] == BSP_PRV_IELS_ENUM(EVENT_GPT4_COUNTER_OVERFLOW)) {
      GPT_IRQn = (IRQn)i;
    }
  }
}

void
DShotR4::dtc_init(void)
{
  assert(GPT_IRQn != FSP_INVALID_VECTOR);

  memset(&dtc_ctrl, 0, sizeof(dtc_ctrl));
  memset(dtc_info, sizeof(dtc_info), 0);
  memset(&dtc_cfg, sizeof(dtc_cfg), 0);
  memset(&dtc_extcfg, sizeof(dtc_extcfg), 0);

  dtc_extcfg.activation_source = GPT_IRQn;

  dtc_cfg.p_info = dtc_info;
  dtc_cfg.p_extend = &dtc_extcfg;

  fsp_err_t err = R_DTC_Open(&dtc_ctrl, &dtc_cfg);
  assert(FSP_SUCCESS == err);
}


/*
 * public
 */
DShotR4::DShotR4(float tr_period, float tr_t1h, float tr_t0h): fsp_timer(), serialCore(this->fsp_timer, this->dtc_ctrl, &this->dtc_info[0], this->dtc_info_len)
{
  this->tolerance_hz = tr_period;
  this->tolerance_t1h = tr_t1h;
  this->tolerance_t0h = tr_t0h;

}

DShotR4::~DShotR4()
{
  end();
}

bool
DShotR4::begin(enum DShotType type, bool biDir, uint8_t channel, bool useChannelA, bool useChannelB, pin_size_t pwmPinA, pin_size_t pwmPinB)
{
  this->dshot_type = type;
  this->gpt_Channel = channel;
  this->gpt_pwmChannelA_enable = useChannelA;
  this->gpt_pwmChannelB_enable = useChannelB;
  this->gpt_pwmPinA = pwmPinA;
  this->gpt_pwmPinB = pwmPinB;
  this->bspPinA = digitalPinToBspPin(pwmPinA);
  this->bspPinB = digitalPinToBspPin(pwmPinB);
  if (biDir && type != DSHOT150) {
    this->dshotInvertA = true;
    this->dshotInvertB = true;
  }
  this->auto_restart = false;

  gpt_init();
  dtc_init();

  // default protocol is DShot.
  gpt_dshot_init();
  dtc_dshot_init();
}

bool
DShotR4::end()
{
  cancel_execution();

  if (dtc_is_open()) {
    R_DTC_Disable(&dtc_ctrl);
    R_DTC_Close(&dtc_ctrl);
    memset(&dtc_ctrl, 0, sizeof(dtc_ctrl));
  }

  if (fsp_timer.is_opened()) {
    fsp_timer.stop();
    fsp_timer.close();
    fsp_timer.end();
  }
}

bool
DShotR4::set_tolerance_hz(float tr_hz)
{
  tolerance_hz = tr_hz;
  gpt_dshot_init();
}

bool
DShotR4::set_tolerance_t1h(float tr_t1h)
{
  tolerance_t1h = tr_t1h;
  gpt_dshot_init();
}

bool
DShotR4::set_tolerance_t0h(float tr_t0h)
{
  tolerance_t0h = tr_t0h;
  gpt_dshot_init();
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
  return set_rawValue(channel, (uint16_t)cmd, (cmd != DSHOT_CMD_DISARM) ? true : false);
}

bool
DShotR4::set_testPattern()
{
  set_rawValue(CHANNEL_A, 0x0555, true);
  set_rawValue(CHANNEL_B, 0x0AAA, false);
  return true;
}

bool
DShotR4::transmit(int count)
{
  if (fsp_timer.is_opened() == false) {
    return false;
  }
  if ((!gpt_pwmChannelA_enable) && (!gpt_pwmChannelB_enable)) {
    return false;
  }
  if (count == 0) {
    // nothing to do.
    return true;
  }

  if (auto_restart && count < 0) {
    nextFrameUpdate = true;
    return true;
  }

  if (count > 0) {
    auto_restart = false;
    auto_restart_count = count;
    tx_start();
    while(tx_busy) {
      yield();
    }
    return true;
  }

  auto_restart = true;
  auto_restart_count = -1;
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

  cancel_execution();

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

  return true;
}

bool
DShotR4::reset()
{
  suspend();
  delay(1000);
  gpt_reg->GTIOR = gtioHigh.gtior;
  delay(1000);
  gpt_reg->GTIOR = gtioLow.gtior;
  delay(1000);
}


