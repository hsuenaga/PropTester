#ifndef __DSHOTR4_H__
#define __DSHOTR4_H__
#include <Arduino.h>
#include <FspTimer.h>

#include "r_dtc.h"

class DShotR4 {
  public:
    enum DShotType {
      DSHOT150 = 0,
      DSHOT300 = 1,
      DSHOT600 = 2,
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

  private:
    enum GTCCR_Map {
      GTCCR_A = 0,
      GTCCR_B,
      GTCCR_C,
      GTCCR_E,
      GTCCR_D,
      GTCCR_F,
    };


    const pin_size_t pin_dshot = 0;
    const pin_size_t pin_dshot_ref = 1;

    const struct {
      float bit_duration_us;
      float t1h_us;
      float t0h_us;
    } dshot_timing[DSHOT_MAX] = {
      {6.67, 5.00, 2.50}, // DSHOT150
      {3.33, 2.50, 1.25}, // DSHOT300
      {1.67, 1.25, 0.625}, // DSHOT600
    };
    
    enum DShotType dshot_type = DSHOT150;
    bool running = false;

    // GPT
    uint32_t period_count;
    uint32_t t1h_count;
    uint32_t t0h_count;
    uint32_t stop_count;
    uint32_t intr_count;
    uint32_t intr_count_total;
    uint32_t dshot_port;
    FspTimer fsp_timer;
    IRQn_Type GPT_IRQn = FSP_INVALID_VECTOR;
    uint32_t gpt_stop_cmd;

    // DTC
    bool enable_dtc;
    dtc_instance_ctrl_t dtc_ctrl;
    dtc_extended_cfg_t dtc_extcfg;
    transfer_info_t dtc_info_template[2];
    transfer_info_t dtc_info[2];
    transfer_cfg_t dtc_cfg;

    // Bit patterns
    uint32_t duty_table[17];
    uint32_t *cur_duty;

    static void dshot_intr0(timer_callback_args_t (*arg));
    void dshot_intr(timer_callback_args_t (*arg));
    void dshot_param_init(void);
    void dshot_pin_init(void);
    void dshot_gpt_init(void);
    void dshot_dtc_init(void);
    void dshot_setup_duty_table(uint16_t frame);

  public:
    DShotR4();
    ~DShotR4();

    bool init(enum DShotType = DSHOT150, bool enable_dtc = false);
    bool send_rawValue(uint16_t value, bool telemetry = false);
    bool send_throttle(uint16_t throttole, bool telemetry = false);
    bool send_command(enum DShotCommand cmd);
    bool start(uint16_t frame);
    bool stop(void);
};

#endif /* __DSHOTR4_H__ */