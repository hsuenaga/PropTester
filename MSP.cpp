#include "MSP.h"

#define U16(x) ((x) & 0xff), (((x) >> 8) & 0xff)
#define U32(x) ((x) & 0xff), (((x) >> 8) & 0xff), (((x) >> 16) & 0xff), (((x) >> 24) & 0xff)

void
MSP::reply(const uint8_t *payload, uint8_t len)
{
  uint8_t buf[256];
  uint8_t *p = buf;
  uint8_t checksum;
  int framelen = len + 6; // 5 bytes header + 1 byte checksum

  if (framelen > sizeof(buf)) {
    error++;
    return;
  }
  *p++ = '$';
  *p++ = 'M';
  *p++ = '>';
  *p++ = len; checksum = len;
  *p++ = command; checksum ^= command;
  for (int i = 0; i < len; i++) {
    *p++ = payload[i];
    checksum ^= payload[i];
  }
  *p = checksum;

  MspPort.write(buf, len);
}

void
MSP::proc_api_version()
{
  uint8_t payload[3] = { MSP_PROTOCOL_VERSION, API_VERSION_MAJOR, API_VERSION_MINOR};

  reply(payload, sizeof(payload));
}

void
MSP::proc_fc_variant()
{
  // fake Ardupilot, we are kind of Ardu*
  reply((uint8_t *)"ARDU", 4);
}

void
MSP::proc_fc_version()
{
  uint8_t payload[3] = {3, 3, 1};
  reply(payload, sizeof(payload));
}

void
MSP::proc_board_info()
{
  // fake ArduPilot ChibiOS
  uint8_t payload[7] = {'A', 'R', 'C', 'H', '\0', '\0', '\0'};
  reply(payload, sizeof(payload));
}

void
MSP::proc_build_info()
{
  // fake ArduPilot ... what is this???
  uint8_t payload[26] {
    0x4d, 0x61, 0x72, 0x20, 0x31, 0x36, 0x20, 0x32, 0x30,
    0x31, 0x38, 0x30, 0x38, 0x3A, 0x34, 0x32, 0x3a, 0x32, 0x39,
    0x62, 0x30, 0x66, 0x66, 0x39, 0x32, 0x38
  };
  reply(payload, sizeof(payload));
}

void
MSP::proc_feature_config()
{
  uint8_t payload[4] = {0, 0, 0, 0};
  reply(payload, sizeof(payload));
}

void
MSP::proc_reboot()
{
  // XXX: shutdown ESC serial communication here.
}

void
MSP::proc_advanced_config()
{
  uint8_t payload[10] = {
    1, // gyro sync denom
    4, // pid process denom
    0, // use unsynced pwm,
    5, // PWM_TYPE_DSHOT150
    U16(480), // motor PWM Rate
    U16(450), // idle offset value
    0, // use 32kHz
    0 // motor PWM inversion
  };
  reply(payload, sizeof(payload));
}

void
MSP::proc_status()
{
  uint8_t payload[21] = {
    U16(1000), // loop time usec
    U16(0), // i2c error count
    U16(0x27), // available sensors
    U32(0), // flight modes
    0, // pid profile index
    U16(5), // system load percent
    U16(0), // gyro cycle time
    0, // flight mode flags length
    18, // arming disable flags count
    U32(0), // arming disable flags
  };
  reply(payload, sizeof(payload));
}

void
MSP::proc_motor()
{
  uint8_t payload[16] = {};
  // XXX: read from ESC
  reply(payload, sizeof(payload));
}

void
MSP::proc_motor_3d_config()
{
  uint8_t payload[6] = {
    U16(1406), // 3D dead band low
    U16(1514), // 3D dead band high
    U16(1460), // 3D neutral    
  };
  reply(payload, sizeof(payload));
}

void
MSP::proc_battery_state()
{
  uint8_t payload[8] = {
    4, // cell count
    U16(1500), // mAh
    16, // V
    U16(1500), // mAh
    U16(1), // A    
  };
  reply(payload, sizeof(payload));
}

void
MSP::proc_motor_config()
{
  uint8_t payload[10] = {
    U16(1030), // min throttle
    U16(2000), // max throttle
    U16(1000), // min command
    1, // motor count
    12, // motor poles
    0, // useDshotTelemetry
    0 // FEATURE_ESC_SENSOR
  };
  reply(payload, sizeof(payload));
}

void
MSP::proc_uid()
{
  const bsp_unique_id_t *id_ptr = R_BSP_UniqueIdGet(); // XXX: RA has 16 bytes identifier
  uint8_t payload[12];
  for (int i = 0; i < 12; i++) {
    payload[i] = id_ptr->unique_id_bytes[i];
  }
  reply(payload, sizeof(payload));
}

void
MSP::proc_set_motor()
{
  // XXX: update motor status.
  reply(NULL, 0);
}

void
MSP::proc_set_passthrough()
{
  uint8_t payload[1] = {
    1 // num motors
  };
  // XXX: setup BLHELI connection.
  reply(payload, sizeof(payload));
}

void
MSP::process()
{  
  static const msp_command_table command_table[16] = {
    {MSP_API_VERSION, &MSP::proc_api_version},
    {MSP_FC_VARIANT, &MSP::proc_fc_variant},
    {MSP_FC_VERSION, &MSP::proc_fc_version},
    {MSP_BOARD_INFO, &MSP::proc_board_info},
    {MSP_BUILD_INFO, &MSP::proc_build_info},
    {MSP_FEATURE_CONFIG, &MSP::proc_feature_config},
    {MSP_REBOOT, &MSP::proc_reboot},
    {MSP_ADVANCED_CONFIG, &MSP::proc_advanced_config},
    {MSP_STATUS, &MSP::proc_status},
    {MSP_MOTOR, &MSP::proc_motor},
    {MSP_MOTOR_3D_CONFIG, &MSP::proc_motor_3d_config},
    {MSP_BATTERY_STATE, &MSP::proc_battery_state},
    {MSP_MOTOR_CONFIG, &MSP::proc_motor_config},
    {MSP_UID, &MSP::proc_uid},
    {MSP_SET_MOTOR, &MSP::proc_set_motor},
    {MSP_SET_PASSTHROUGH, &MSP::proc_set_passthrough}
  };

  for (int i = 0; i < sizeof(command_table)/sizeof(command_table[0]); i++) {
    if (command != command_table[i].code) {
      continue;
    }
    if (command_table[i].exec != NULL) {
      (this->*command_table[i].exec)();
      return;
    }
  }

  // silent discard.
  return;
}

MSP::MSP(Stream &Port, DShotR4 &DShotInstance) : MspPort(Port), DShot(DShotInstance)
{
}

MSP::~MSP()
{
}

bool
MSP::receive(int byte)
{
  switch (state) {
    case MSP_IDLE:
      if (byte == '$') {
        state = MSP_HEADER_START;
        return true;
      }
      break;
    case MSP_HEADER_START:
      if (byte == 'M') {
        state = MSP_HEADER_M;
        return true;
      }
      break;
    case MSP_HEADER_M:
      if (byte == '<') {
        type = MSP_PACKET_COMMAND;
        state = MSP_HEADER_ARROW;
        return true;
      }
      else if (byte == '>') {
        type = MSP_PACKET_REPLY;
        state = MSP_HEADER_ARROW;
        return true;
      }
      break;
    case MSP_HEADER_ARROW:
      if (byte > sizeof(data)) {
        break;
      }
      dataSize = byte;
      dataOffset = 0;
      dataChecksum = byte;
      state = MSP_HEADER_SIZE;
      return true;
    case MSP_HEADER_SIZE:
      command = byte;
      dataChecksum ^= byte;
      state = MSP_HEADER_CMD;
      return true;
    case MSP_HEADER_CMD:
      if (dataOffset < dataSize) {
        data[dataOffset++] = byte;
        dataChecksum ^= byte;
        return true;
      }
      if (dataChecksum == byte) {
        received++;
        process();
        state = MSP_IDLE;
        return true;
      }
      break;
    default:
      break;
  }

  error++;
  state = MSP_IDLE;
  return false;
}