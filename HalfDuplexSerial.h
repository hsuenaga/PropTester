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
	R_GPT0_Type *gpt_reg;
	uint32_t gpt_stopVal;

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

	const static int serialTxBits = 12; // idle(1) + start(1) + data(8) + stop(1) + idle(1)
	uint8_t serialTxLevel[serialTxBits];
	uint8_t serialTxDebug[serialTxBits];
	const uint8_t *serialTxPtr;
	int serialTxBytes;

	const static int serialRxBits = 10; // start(1) + data(8) + stop(1)
	uint8_t serialRxLevel[serialRxBits];
	uint8_t serialRxDebug[serialRxBits];
	const static int serialRxFIFOLen = 16;
	uint8_t serialRxFIFO[serialRxFIFOLen];
	uint8_t *serialRxFIFO_IN;
	uint8_t *serialRxFIFO_OUT;
	int serialRxBytes;
	uint8_t serialRxBuff[serialRxFIFOLen];
	uint8_t *serialRxBuff_OUT;
	int serialRxBuff_Remain;

	// external instances
	FspTimer &fsp_timer;
	dtc_instance_ctrl_t &dtc_ctrl;
	transfer_info_t *dtc_info;
	size_t dtc_info_len;

	// ELC
	elc_instance_ctrl_t elc_ctrl;
	void elc_link(int port);

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
	uint32_t tx_busy;
	uint32_t tx_serial_success;
	uint32_t rx_serial_detect;
	uint32_t rx_serial_good_frames;
	uint32_t rx_serial_bad_frames;
	bool tx_serial = false;
	bool rx_serial = false;

	HalfDuplexSerialCore(FspTimer &timer, dtc_instance_ctrl_t &dtc, transfer_info_t *info);
	~HalfDuplexSerialCore();

	void begin(TimerPWMChannel_t ch, pin_size_t pin, uint32_t bps = 19200);
	void end();

	size_t write(uint8_t) override;
	size_t write(const uint8_t *buffer, size_t size) override;
	int availableForWrite() override;
	void flush(void) override;

	int available(void) override;
	int read(void) override;
	int peek(void) override;

	void tx_serial_complete(timer_callback_args_t(*arg));
	void rx_serial_complete(timer_callback_args_t(*arg));

	bool get_rx_raw_buff(uint8_t *dst, size_t *len)
	{
		if (*len < serialRxBits)
		{
			*len = serialRxBits;
			return false;
		}
		memcpy(dst, serialRxLevel, serialRxBits);
		*len = serialRxBits;
		return true;
	};

	bool get_rx_debug_buff(uint8_t *dst, size_t *len)
	{
		if (*len < serialRxBits)
		{
			*len = serialRxBits;
			return false;
		}
		memcpy(dst, serialRxDebug, serialRxBits);
		*len = serialRxBits;
		return true;
	};
};
