#ifndef __DSHOTR4_H__
#define __DSHOTR4_H__
#include <Arduino.h>
#include <FspTimer.h>

#include "r_gpt.h"
#include "r_dtc.h"

class DShotR4 {
  public:
    enum DShotType {
      DSHOT150 = 0,
      DSHOT300 = 1,
      DSHOT600 = 2,
      DSHOT1200 = 3,
      DSHOT_MAX
    };

    enum DShotCommand {
      DSHOT_CMD_DISARM = 0,
      DSHOT_CMD_BEEP_1 = 1, // Low freq.
      DSHOT_CMD_BEEP_2 = 2,
      DSHOT_CMD_BEEP_3 = 3,
      DSHOT_CMD_BEEP_4 = 4,
      DSHOT_CMD_BEEP_5 = 5, // High freq.
      DSHOT_CMD_INFO_REQ = 6,
      DSHOT_CMD_DIRECTION_1 = 7,
      DSHOT_CMD_DIRECTION_2 = 8,
      DSHOT_CMD_3D_MODE_OFF = 9,
      DSHOT_CMD_3D_MODE_ON = 10,
      DSHOT_CMD_SETTING_REQ = 11,
      DSHOT_CMD_SETTING_SAVE = 12,
      DSHOT_CMD_EDT_ENABLE = 13,
      DSHOT_CMD_EDT_DISABLE = 14,
      DSHOT_CMD_DIRECTION_NORMAL = 20,
      DSHOT_CMD_DIRECTION_REVERSED = 21,
      DSHOT_CMD_LED_0_ON = 22,
      DSHOT_CMD_LED_1_ON = 23,
      DSHOT_CMD_LED_2_ON = 24,
      DSHOT_CMD_LED_3_ON = 25,
      DSHOT_CMD_LED_0_OFF = 26,
      DSHOT_CMD_LED_1_OFF = 27,
      DSHOT_CMD_LED_2_OFF = 28,
      DSHOT_CMD_LED_3_OFF = 29,
      DSHOT_CMD_AUDIO_STREAM_TOGGLE = 30,
      DSHOT_CMD_SILENT_MODE_TOGGLE = 31,
      DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE = 32,
      DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY = 33,
      DSHOT_CMD_MAX = 47,
    };
    uint32_t tx_success;
    uint32_t tx_error;

  private:
    enum GTCCR_Map {
      GTCCR_A = 0,
      GTCCR_B,
      GTCCR_C,
      GTCCR_E,
      GTCCR_D,
      GTCCR_F,
    };

    const struct {
      float freqHz;
      float bit_duration_us;
      float t1h_us;
      float t0h_us;
    } dshot_timing[DSHOT_MAX] = {
      {150 * 1000, 6.67, 5.00, 2.50}, // DSHOT150
      {300 * 1000, 3.33, 2.50, 1.25}, // DSHOT300
      {600 * 1000, 1.67, 1.25, 0.625}, // DSHOT600
      {1200 * 1000, 0.83, 0.625, 0.313} // DSHOT1200
    };
    enum DShotType dshot_type;
    uint32_t period_count;
    uint32_t t1h_count;
    uint32_t t0h_count;
    uint32_t stop_count;

    // Bit patterns
    bool dshotInvertA = false;
    bool dshotInvertB = false;
    uint16_t dshotFrame[2];
    uint32_t duty_table[2][17];

    // GPT
    bool tx_busy = false;
    FspTimer fsp_timer;
    uint8_t gpt_Channel;
    bool gpt_pwmChannelA_enable;
    bool gpt_pwmChannelB_enable;
    pin_size_t gpt_pwmPinA;
    pin_size_t gpt_pwmPinB;
    IRQn_Type GPT_IRQn = FSP_INVALID_VECTOR;
    R_GPT0_Type *gpt_reg;

    // DTC
    dtc_instance_ctrl_t dtc_ctrl;
    transfer_cfg_t dtc_cfg;
    dtc_extended_cfg_t dtc_extcfg;
    transfer_info_t dtc_info[3];

    static void gpt_overflow_intr(timer_callback_args_t (*arg));
    void tx_complete(timer_callback_args_t (*arg));
    void timing_init(void);
    void gpt_gtioA_init(gpt_extended_cfg_t (*ext_cfg));
    void gpt_gtioB_init(gpt_extended_cfg_t (*ext_cfg));
    void gpt_init();

    void dtc_info_init(transfer_info_t (*info));
    void dtc_info_reset(transfer_info_t (*info));
    void dtc_init(void);

    void make_duty_counts(uint32_t table[], uint16_t frame);
    bool tx_start();

  public:
    DShotR4();
    ~DShotR4();

    bool init(enum DShotType = DSHOT150, bool biDir = false, uint8_t channel = 4, bool useChannelA = true, bool useChannelB = true, pin_size_t pwmPinA = 0, pin_size_t pwmPinB = 1);
    bool set_rawValue(TimerPWMChannel_t channel, uint16_t value, bool telemetry = false);
    bool set_throttle(TimerPWMChannel_t channel, uint16_t throttole, bool telemetry = false);
    bool set_command(TimerPWMChannel_t channel, enum DShotCommand cmd);
    bool set_testPattern(void);
    bool transmit(void);
};

#endif /* __DSHOTR4_H__ */