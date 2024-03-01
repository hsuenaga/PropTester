#ifndef __DSHOTR4_H__
#define __DSHOTR4_H__
#include <Arduino.h>
#include <FspTimer.h>

#include "r_gpt.h"
#include "r_dtc.h"
#include "r_elc.h"
#include "r_elc_api.h"
#include "elc_defines.h"

#undef DEBUG_CHANNEL_A

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
    uint32_t suprious_intr;
    uint32_t tx_success;
    uint32_t tx_error;
    uint32_t tx_serial_success;
    uint32_t rx_serial_detect;
    uint32_t rx_serial_good_frames;
    uint32_t rx_serial_bad_frames;

  private:
    enum GTCCR_Map {
      GTCCR_A = 0,
      GTCCR_B,
      GTCCR_C,
      GTCCR_E,
      GTCCR_D,
      GTCCR_F,
    };

    // see variants/MINIMA/includes/ra/fsp/inc/api/r_ioport_api.h
    static const uint32_t pin_cfg_output_gpt = 
      IOPORT_CFG_PORT_DIRECTION_OUTPUT |
      IOPORT_CFG_DRIVE_MID |
      IOPORT_CFG_PERIPHERAL_PIN |
      IOPORT_PERIPHERAL_GPT1;

    static const uint32_t pin_cfg_output_high = 
      IOPORT_CFG_PORT_DIRECTION_OUTPUT |
      IOPORT_CFG_PORT_OUTPUT_HIGH |
      IOPORT_CFG_DRIVE_MID;

    static const uint32_t pin_cfg_output_low = 
      IOPORT_CFG_PORT_DIRECTION_OUTPUT |
      IOPORT_CFG_PORT_OUTPUT_LOW |
      IOPORT_CFG_DRIVE_MID;

    static const uint32_t pin_cfg_input =
      IOPORT_CFG_PORT_DIRECTION_INPUT |
      IOPORT_CFG_PULLUP_ENABLE |
      IOPORT_CFG_DRIVE_MID |
      IOPORT_CFG_EVENT_FALLING_EDGE;

    struct GTIO_PIN_MAP {
      int channel;
      pin_size_t gtio_A;
      pin_size_t gtio_B;
    };
#if defined(ARDUINO_MINIMA)
    struct GTIO_PIN_MAP pinMap[8] = {
      {0, 7, 6},    // P107, P106
      {1, 2, 3},    // P105, P104
      {1, 11, 12},  // P109, P110
      {2, 4, 5},    // P103, P102
      {3, 13, 10},  // P111, P112
      {4, 1, 0},    // P302, P301
      {5, A4, A5},  // P101, P100
      {7, 8, 9},    // P304, P303
    };
#elif defined(ARDUINO_UNOWIFIR4)
    struct GTIO_PIN_MAP pinMap[8] = {
      {0, 5, 4},    // P107, P106
      {1, 3, 2},    // P105, P104
      {2, 10, ~0},    // P103, P102(J3.pin3)
      {3, 6, 7},  // P111, P112
      {4, 1, 0},    // P302, P301
      {5, A4, A5},  // P101, P100
      {7, 8, 9},    // P304, P303
    };
#else
#error "Board is not supported."
#endif
    inline int pin2gptChannel(pin_size_t pin) {
      for (int i = 0; sizeof(pinMap)/sizeof(pinMap[0]); i++) {
        if (pinMap[i].gtio_A == pin || pinMap[i].gtio_B == pin) {
          return pinMap[i].channel;
        }
      }
      return -1;
    };

    inline int pin2pwmChannel(pin_size_t pin) {
      for (int i = 0; sizeof(pinMap)/sizeof(pinMap[0]); i++) {
        if (pinMap[i].gtio_A == pin) {
          return (int)CHANNEL_A;
        }
        if (pinMap[i].gtio_B == pin) {
          return (int)CHANNEL_B;
        }
      }
      return -1;
    }

    const struct dshot_params_t {
      float bit_duration_us;
      float t1h_us;
      float t0h_us;
    } dshot_timing[DSHOT_MAX] = {
      {6.67, 5.00, 2.50}, // DSHOT150
      {3.33, 2.50, 1.25}, // DSHOT300
      {1.67, 1.25, 0.625}, // DSHOT600
      {0.83, 0.625, 0.313} // DSHOT1200
    };
    float tolerance_hz;
    float tolerance_t1h;
    float tolerance_t0h;

    const uint8_t blheli_signature[21] = {
      0,0,0,0,0,0,0,0,0,0,0,0,0x0D,
      'B','L','H','e','l','i',
      0xF4,0x7D
    };

    enum DShotType dshot_type;
    uint32_t period_count;
    uint32_t t1h_count;
    uint32_t t0h_count;
    uint32_t stop_count;
    uint32_t dshot_ifg_us;
    bool auto_restart;
    int auto_restart_count;
    inline void wait_execution(void) {
      while (tx_busy || auto_restart_count > 0) {
        yield();
      }
    };
    inline void cancel_execution(void) {
      noInterrupts();
      auto_restart = false;
      auto_restart_count = -1;
      interrupts();
      wait_execution();
    };

    // Bit patterns
    bool dshotInvertA = false;
    bool dshotInvertB = false;
    uint16_t nextFrame[2];
    bool nextFrameUpdate = false;
    bool bootloader = false;
    const static int waveformBits = 17;
    const static int ifgBits = waveformBits * 2; // XXX: what is reasonable value???
    const static int dshotBits = waveformBits + ifgBits;
    uint32_t waveform[2][dshotBits];
    uint32_t gtioState[dshotBits];

    const static int serialTxBits = 12; // idle(1) + start(1) + data(8) + stop(1) + idle(1)
    uint8_t serialTxLevel[2][serialTxBits];
    uint8_t serialTxDebug[serialTxBits];
    const uint8_t *serialTxPtr;
    int serialTxBytes;

    const static int serialRxBits = 10; // start(1) + data(8) + stop(1)
    uint8_t serialRxLevel[2][serialRxBits];
    uint8_t serialRxDebug[serialRxBits];
    const static int serialRxFIFOLen = 16;
    uint8_t serialRxFIFO[serialRxFIFOLen];
    uint8_t *serialRxFIFO_IN;
    uint8_t *serialRxFIFO_OUT;
    int serialRxBytes;
    uint8_t serialRxBuff[serialRxFIFOLen];
    uint8_t *serialRxBuff_OUT;
    int serialRxBuff_Remain;

    // GPT
    bool tx_busy = false;
    bool tx_serial = false;
    bool rx_serial = false;
    FspTimer fsp_timer;
    uint8_t gpt_Channel;
    uint32_t gpt_ChannelRegVal;
    bool gpt_pwmChannelA_enable;
    bool gpt_pwmChannelB_enable;
    pin_size_t gpt_pwmPinA;
    pin_size_t gpt_pwmPinB;
    bsp_io_port_pin_t bspPinA;
    bsp_io_port_pin_t bspPinB;
    IRQn_Type GPT_IRQn = FSP_INVALID_VECTOR;
    R_GPT0_Type *gpt_reg;
    gpt_gtior_setting_t gtioHigh;
    gpt_gtior_setting_t gtioLow;
    gpt_gtior_setting_t gtioRunning;
    gpt_gtior_setting_t gtioStop;

    // DTC
    dtc_instance_ctrl_t dtc_ctrl;
    transfer_cfg_t dtc_cfg;
    dtc_extended_cfg_t dtc_extcfg;
    transfer_info_t dtc_dshot_info[5];
    transfer_info_t dtc_serial_info[4];
    inline bool dtc_is_open() {
      transfer_properties_t dtc_prop;

      if (R_DTC_InfoGet(&dtc_ctrl, &dtc_prop) == FSP_SUCCESS) {
        return true;
      }
      return false;
    }

    // ELC
    elc_instance_ctrl_t elc_ctrl;

    static void gpt_overflow_intr(timer_callback_args_t (*arg));
    void tx_dshot_complete(timer_callback_args_t (*arg));
    void tx_serial_complete(timer_callback_args_t (*arg));
    void rx_serial_complete(timer_callback_args_t (*arg));

    void update_timer_counts(void);
    void gpt_gtioState_init(void);
    void gpt_init();
    void gpt_dshot_init();
    void gpt_serial_init();

    void dtc_dshot_info_init(transfer_info_t (*info));
    void dtc_dshot_info_reset(transfer_info_t (*info));
    void dtc_serial_info_tx_init(transfer_info_t (*info));
    void dtc_serial_info_tx_reset(transfer_info_t (*info));
    void dtc_serial_info_rx_init(transfer_info_t (*info));
    void dtc_serial_info_rx_reset(transfer_info_t (*info));
    void dtc_dshot_init(void);
    void dtc_serial_init(void);

    void elc_link(int port);

    void load_dshot_frame(void);
    void load_serial_frame(TimerPWMChannel_t ch = CHANNEL_B);
    bool tx_restart(void);
    bool tx_start(void);
    bool tx_serial_restart(void);
    bool tx_serial_start(const uint8_t *data, size_t len);
    bool rx_serial_restart(bool initial = false);
    bool rx_serial_start(void);

  public:
    DShotR4(float tr_hz = 1.05, float tr_t1h = 1.05, float tr_t0h = 0.95);
    ~DShotR4();

    bool init(enum DShotType = DSHOT150, bool biDir = false, uint8_t channel = 4, bool useChannelA = true, bool useChannelB = true, pin_size_t pwmPinA = 1, pin_size_t pwmPinB = 0);
    bool deinit();

    bool set_tolerance_hz(float tr_hz);
    bool set_tolerance_t1h(float tr_t1h);
    bool set_tolerance_t0h(float tr_t0h);

    bool set_rawValue(TimerPWMChannel_t channel, uint16_t value, bool telemetry = false);
    bool set_throttle(TimerPWMChannel_t channel, uint16_t throttole, bool telemetry = false);
    bool set_command(TimerPWMChannel_t channel, enum DShotCommand cmd);
    bool set_testPattern(void);

    bool transmit(int count = -1);
    bool suspend(void);
    bool arm(void);
    bool reset(void);

    bool bl_enter(void);
    bool bl_exit(void);
    bool bl_open();
    int bl_read();
    int bl_peek();
    int bl_flush();
    int bl_write(uint8_t data);

    uint32_t get_ifg_us() {
      return dshot_ifg_us;
    };

    bool get_auto_restart() {
      return auto_restart;
    };

    int get_auto_restart_count() {
      return auto_restart_count;
    };

    uint8_t get_channel() {
      return gpt_Channel;
    };

    bsp_io_port_pin_t get_bspPinA() {
      return bspPinA;
    };

    bsp_io_port_pin_t get_bspPinB() {
      return bspPinB;
    };

    pin_size_t get_pwmPinA() {
      return gpt_pwmPinA;
    };

    pin_size_t get_pwmPinB() {
      return gpt_pwmPinB;
    };

    bool get_bl_rx_raw_buff(int ch, uint8_t *dst, size_t *len) {
      if (*len < serialRxBits) {
        *len = serialRxBits;
        return false;
      }
      memcpy(dst, serialRxLevel[ch], serialRxBits);
      *len = serialRxBits;
      return true;
    };

    bool get_bl_rx_debug_buff(uint8_t *dst, size_t *len) {
      if (*len < serialRxBits) {
        *len = serialRxBits;
        return false;
      }
      memcpy(dst, serialRxDebug, serialRxBits);
      *len = serialRxBits;
      return true;
    };
};

#endif /* __DSHOTR4_H__ */