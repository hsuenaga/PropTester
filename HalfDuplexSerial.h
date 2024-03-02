#pragma once

#include <Arduino.h>
#include <FspTimer.h>

#include "r_gpt.h"
#include "r_dtc.h"
#include "r_elc.h"
#include "r_elc_api.h"
#include "elc_defines.h"

class HalfDuplexSerialCore : public Stream
{
private:
	TimerPWMChannel_t channel;
	pin_size_t digitalPin;
	uint32_t bitRate;
	bsp_io_port_pin_t bspPin;
	int bspPort;
	int bspPinOffset;
	R_GPT0_Type *gptReg;
	uint32_t pinCfgSave;

	static const uint32_t pinCfgOutputHigh =
	    IOPORT_CFG_PORT_DIRECTION_OUTPUT |
	    IOPORT_CFG_PORT_OUTPUT_HIGH |
	    IOPORT_CFG_DRIVE_MID;

    static const uint32_t pinCfgOutputLow =
      IOPORT_CFG_PORT_DIRECTION_OUTPUT |
      IOPORT_CFG_PORT_OUTPUT_LOW |
      IOPORT_CFG_DRIVE_MID;

	static const uint32_t pinCfgInput =
	    IOPORT_CFG_PORT_DIRECTION_INPUT |
	    IOPORT_CFG_PULLUP_ENABLE |
	    IOPORT_CFG_DRIVE_MID |
	    IOPORT_CFG_EVENT_FALLING_EDGE;

	const static int txBits = 12; // idle(1) + start(1) + data(8) + stop(1) + idle(1)
	uint8_t txPFSBY[txBits];
	uint8_t txDebug[txBits];
	const uint8_t *txPtr;
	int txBytes;

	const static int rxBits = 10; // start(1) + data(8) + stop(1)
	uint8_t rxPFSBY[rxBits];
	uint8_t rxDebug[rxBits];
	const static int rxFIFOLen = 16;
	uint8_t rxFIFO[rxFIFOLen];
	uint8_t *rxFIFO_IN;
	uint8_t *rxFIFO_OUT;
	int rxFIFO_Bytes;
	uint8_t rxBuffer[rxFIFOLen];
	uint8_t *rxBuffer_OUT;
	int rxBuffer_Bytes;

	// external instances
	FspTimer &fspTimer;
	dtc_instance_ctrl_t &dtcCtrl;
	transfer_info_t *dtcInfo;
	size_t dtcInfoLen;

	// ELC
	elc_instance_ctrl_t elcCtrl;
	void elc_link(int port);

	// interrupt handlers
	void tx_serial_complete(timer_callback_args_t(*arg));
	void rx_serial_complete(timer_callback_args_t(*arg));

	void gpt_serial_init();
	void dtc_serial_info_tx_init(void);
	void dtc_serial_info_tx_reset(void);
	void dtc_serial_info_rx_init(void);
	void dtc_serial_info_rx_reset(void);
	void dtc_serial_init(void);

	void load_serial_frame(TimerPWMChannel_t ch = CHANNEL_B);
	void tx_abort(void);
	bool tx_serial_restart(void);
	bool tx_serial_start(const uint8_t *data, size_t len);
	bool rx_serial_restart(bool initial = false);
	bool rx_serial_start(void);

public:
	struct HalfDuplexSerialCoreCounter_t {		
		uint32_t tx_success;
		uint32_t rx_detect;
		uint32_t rx_good_frames;
		uint32_t rx_bad_frames;
		uint32_t spurious_interrupts;
	} Counter;
	bool tx_busy;
	bool tx_serial = false;
	bool rx_serial = false;

	HalfDuplexSerialCore(FspTimer &timer, dtc_instance_ctrl_t &dtc, transfer_info_t *info);
	~HalfDuplexSerialCore();

	void overflow_interrupt(timer_callback_args_t(*arg));

	void begin(TimerPWMChannel_t ch, pin_size_t pin, uint32_t bps = 19200);
	void end();

	size_t write(uint8_t) override;
	size_t write(const uint8_t *buffer, size_t size) override;
	int availableForWrite() override;
	void flush(void) override;

	int available(void) override;
	int read(void) override;
	int peek(void) override;


	bool get_rx_raw_buff(uint8_t *dst, size_t *len)
	{
		if (*len < rxBits)
		{
			*len = rxBits;
			return false;
		}
		memcpy(dst, rxPFSBY, rxBits);
		*len = rxBits;
		return true;
	};

	bool get_rx_debug_buff(uint8_t *dst, size_t *len)
	{
		if (*len < rxBits)
		{
			*len = rxBits;
			return false;
		}
		memcpy(dst, rxDebug, rxBits);
		*len = rxBits;
		return true;
	};
};
