#pragma once
#ifndef __HALFDUPLEXSERIAL_H__
#define __HALFDUPLEXSERIAL_H__
#include <Arduino.h>
#include <FspTimer.h>

#include <vector>

#include "elc_defines.h"
#include "r_dtc.h"
#include "r_elc.h"
#include "r_elc_api.h"
#include "r_gpt.h"

class HalfDuplexSerialCore : public Stream {
 public:
  struct Counter_t {
    uint32_t tx_success;
    uint32_t rx_detect;
    uint32_t rx_overflow;
    uint32_t rx_good_frames;
    uint32_t rx_bad_frames;
    uint32_t spurious_interrupts;
  };

  struct Buffer_t {
    uint32_t txFIFO_Max;
    uint32_t txFIFO_Len;
    uint32_t rxFIFO_Max;
    uint32_t rxFIFO_Len;
  };

  struct Config_t {
    bool invert;
    bool opendrain;
  };

 private:
  TimerPWMChannel_t channel;
  pin_size_t digitalPin;
  uint32_t bitRate;
  bsp_io_port_pin_t bspPin;
  int bspPort;
  int bspPinOffset;
  R_GPT0_Type *gptReg;
  uint32_t pinCfgSave;
  bool isOpen;
  Counter_t counter;
  Buffer_t buffer;
  Config_t config;

  static const uint32_t pinCfgOutputHighCMOS = IOPORT_CFG_PORT_DIRECTION_OUTPUT |
                                               IOPORT_CFG_PORT_OUTPUT_HIGH |
                                               IOPORT_CFG_DRIVE_MID;

  static const uint32_t pinCfgOutputLowCMOS = IOPORT_CFG_PORT_DIRECTION_OUTPUT |
                                              IOPORT_CFG_PORT_OUTPUT_LOW |
                                              IOPORT_CFG_DRIVE_MID;

  static const uint32_t pinCfgOutputHighNMOS = IOPORT_CFG_PORT_DIRECTION_OUTPUT |
                                               IOPORT_CFG_PORT_OUTPUT_HIGH |
                                               IOPORT_CFG_NMOS_ENABLE |
                                               IOPORT_CFG_DRIVE_MID;

  static const uint32_t pinCfgOutputLowNMOS = IOPORT_CFG_PORT_DIRECTION_OUTPUT |
                                              IOPORT_CFG_PORT_OUTPUT_LOW |
                                              IOPORT_CFG_NMOS_ENABLE |
                                              IOPORT_CFG_DRIVE_MID;

  static const uint32_t pinCfgInputPU = IOPORT_CFG_PORT_DIRECTION_INPUT |
                                        IOPORT_CFG_PULLUP_ENABLE |
                                        IOPORT_CFG_DRIVE_MID |
                                        IOPORT_CFG_EVENT_FALLING_EDGE;

  static const uint32_t pinCfgInputHZ = IOPORT_CFG_PORT_DIRECTION_INPUT |
                                        IOPORT_CFG_DRIVE_MID |
                                        IOPORT_CFG_EVENT_FALLING_EDGE;

  uint32_t pinCfgOutput1;
  uint32_t pinCfgOutput0;
  uint32_t pinCfgInput;

  bool tx_busy = false;
  const static int txBits = 11;  // idle/stop(1) + start(1) + data(8) + stop(<1)
  uint32_t txIFG;
  uint32_t gptStartStopVal;
  using txPFSBY_t = uint8_t[txBits];
  const static int txFIFOLen = 16;
  txPFSBY_t txFIFO[txFIFOLen];
  txPFSBY_t *txFIFO_IN;
  txPFSBY_t *txFIFO_OUT;
  int txFIFO_Bytes;

  bool rx_ready = false;
  bool rx_overflow = false;
  uint32_t rxCapture;
  const static int rxBits = 10;  // start(1) + data(8) + stop(1)
  using rxPFSBY_t = uint8_t[rxBits];
  const static int rxFIFOLen = 16;
  rxPFSBY_t rxFIFO[rxFIFOLen];
  rxPFSBY_t *rxFIFO_IN;
  rxPFSBY_t *rxFIFO_OUT;
  int rxFIFO_Bytes;

  // external instances
  FspTimer &fspTimer;
  dtc_instance_ctrl_t &dtcCtrl;
  transfer_info_t *dtcInfo;
  size_t dtcInfoLen;

  // ELC
  elc_instance_ctrl_t elcCtrl;

  // interrupt handlers
  void tx_serial_complete(timer_callback_args_t(*arg));
  void rx_serial_complete(timer_callback_args_t(*arg));

  void gpt_init();

  void elc_link(int port);

  void dtc_init(void);
  void dtc_info_tx_init(void);
  void dtc_info_tx_reset(void);
  void dtc_info_rx_init(void);
  void dtc_info_rx_reset(void);

  void tx_encode(uint8_t byte, txPFSBY_t &pfsby);
  void tx_abort(void);
  bool tx_serial_restart(bool initial = false);
  bool tx_serial_start();

  int rx_decode(rxPFSBY_t &pfsby);
  bool rx_serial_restart(bool initial = false);
  bool rx_serial_start(void);

 public:
  HalfDuplexSerialCore(FspTimer &timer, dtc_instance_ctrl_t &dtc,
                       transfer_info_t *info, size_t infoLen);
  ~HalfDuplexSerialCore();

  void overflow_interrupt(timer_callback_args_t(*arg));

  void configure(Config_t config);
  void begin(TimerPWMChannel_t ch, pin_size_t pin, uint32_t bps = 19200,
             bool inv = false);
  void end();

  size_t write(uint8_t) override;
  size_t write(const uint8_t *buffer, size_t size) override {
    Print::write(buffer, size);
  };
  int availableForWrite() override;
  void flush(void) override;

  int available(void) override;
  int read(void) override;
  int peek(void) override;

  bool is_open() { return isOpen; }

  bool is_tx_busy(void) { return tx_busy; }

  bool is_rx_ready(void) { return rx_ready; }

  Counter_t get_counter(void) { return counter; }

  Buffer_t get_buffer(void) {
    buffer.txFIFO_Len = txFIFO_Bytes;
    buffer.rxFIFO_Len = rxFIFO_Bytes;
    return buffer;
  }

  Config_t get_config(void) {
    return config;
  }
};
#endif /* __HALFDUPLEXSERIAL_H__ */
