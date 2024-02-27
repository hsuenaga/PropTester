#include <Arduino.h>

#include "DShotR4.h"

class MSP {
  private:
    // $M<SC... or $M>SC...
    enum msp_state_t {
      MSP_IDLE,
      MSP_HEADER_START,
      MSP_HEADER_M,
      MSP_HEADER_ARROW,
      MSP_HEADER_SIZE,
      MSP_HEADER_CMD,
    } state;

    enum msp_packet_type_t {
      MSP_PACKET_COMMAND,
      MSP_PACKET_REPLY
    } type;

    enum msp_command_t {
      MSP_API_VERSION     = 1,
      MSP_FC_VARIANT      = 2,
      MSP_FC_VERSION      = 3,
      MSP_BOARD_INFO      = 4,
      MSP_BUILD_INFO      = 5,
      MSP_FEATURE_CONFIG  = 36,
      MSP_REBOOT          = 68,
      MSP_ADVANCED_CONFIG = 90,
      MSP_STATUS          = 101,
      MSP_MOTOR           = 104,
      MSP_MOTOR_3D_CONFIG = 124,
      MSP_BATTERY_STATE   = 130,
      MSP_MOTOR_CONFIG    = 131,
      MSP_UID             = 160,
      MSP_SET_MOTOR       = 214,
      MSP_SET_PASSTHROUGH = 245,
    };

    struct msp_command_table {
      enum msp_command_t code;
      void (MSP::*exec)(void);
    };

    const uint8_t MSP_PROTOCOL_VERSION = 0;
    const uint8_t API_VERSION_MAJOR = 1;
    const uint8_t API_VERSION_MINOR = 42;
    const uint8_t API_VERSION_LENGTH = 2;

    uint8_t data[256];
    uint8_t dataSize;
    uint8_t dataOffset;
    uint8_t dataChecksum;
    uint8_t command;

    Stream &MspPort;
    DShotR4 &DShot;

    void proc_api_version(void);
    void proc_fc_variant(void);
    void proc_fc_version(void);
    void proc_board_info(void);
    void proc_build_info(void);
    void proc_feature_config(void);
    void proc_reboot(void);
    void proc_advanced_config(void);
    void proc_status(void);
    void proc_motor(void);
    void proc_motor_3d_config(void);
    void proc_battery_state(void);
    void proc_motor_config();
    void proc_uid(void);
    void proc_set_motor(void);
    void proc_set_passthrough(void);

    void reply(const uint8_t *data, uint8_t len);
    void process(void);

  public:
    int received = 0;
    int error = 0;

    MSP(Stream &Port, DShotR4 &DShotInstance);
    ~MSP(void);

    bool receive(int byte);
};