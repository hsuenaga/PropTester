struct shell_command_table {
  const char *name;
  bool (*exec)(char *);
};

struct shell_command_table shell_command[] = {{"reg", exec_reg},
                                              {"stat", exec_stat},
                                              {"arm", exec_arm},
                                              {"reset", exec_reset},
                                              {"suspend", exec_suspend},
                                              {"disarm", exec_disarm},
                                              {"throttle", exec_throttle},
                                              {"commit", exec_commit},
                                              {"bootloader", exec_bootloader},
                                              {"servo", exec_servo},
                                              {"help", exec_help},
                                              {NULL, NULL}};

struct shell_command_table bootloader_command[] = {
    {"reg", exec_reg},
    {"stat", exec_stat},
    {"buffer", exec_buffer},
    {"open", exec_bl_open},
    {"bootinfo", exec_bl_bootinfo},
    {"firminfo", exec_bl_firminfo},
    {"statusinfo", exec_bl_status},
    {"keepalive", exec_bl_keepalive},
    {"setaddress", exec_bl_setaddress},
    {"readflash", exec_bl_readflash},
    {"readeeprom", exec_bl_readeeprom},
    {"readsram", exec_bl_readsram},
    {"restart", exec_bl_restart},
    {"write", exec_bl_write},
    {"exit", exec_bl_exit},
    {NULL, NULL}};

struct shell_command_table serial_servo_command[] = {
    {"version", exec_servo_version},
    {"baud", exec_servo_baud},
    {"setbaud", exec_servo_setbaud},
    {"position", exec_servo_position},
    {"setposition", exec_servo_setposition},
    {"linkpwm", exec_servo_linkpwm},
    {"config", exec_servo_config},
    {"status", exec_servo_status},
    {"reg", exec_reg},
    {"stat", exec_stat},
    {"buffer", exec_buffer},
    {"exit", exec_servo_exit},
    {NULL, NULL}};

struct shell_command_table *current_table = shell_command;
uint16_t last_address = 0;

static void hexdump(const uint8_t *buf, size_t len, size_t column = 16) {
  bool nr = false;
  for (int i = 0; i < len; i++) {
    nr = false;
    if (i % column == 0) {
      message("0x%04x: ", (last_address + i));
    }
    message("%02x", buf[i]);
    if (i % column == (column - 1)) {
      message("\n");
      nr = true;
    }
  }
  if (!nr) {
    message("\n");
  }
}

unsigned long GetPWM() {
  unsigned long timeout = 100 * 1000UL;  // 100[ms]
  unsigned long highTime = pulseIn(pwm_input, HIGH, timeout);

  if (highTime == 0) {
    message("PWM read error.\n");
    return 0;
  }

  return highTime;
}

const char *bl_strerror(uint8_t code) {
  switch (code) {
    case BLHeli::SUCCESS:
      return "SUCCESS";
    case BLHeli::ERRORVERIFY:
      return "ERRORVERIFY";
      break;
    case BLHeli::ERRORCOMMAND:
      return "ERRORCOMMAND";
    case BLHeli::ERRORCRC:
      return "ERRORCRC";
    case BLHeli::NONE:
      return "internal error";
    default:
      break;
  }

  return "UNKNOWN";
}

bool exec_reg(char *arg) {
  uint8_t ch = DShot.get_channel();
  int bsp_portA = DShot.get_bspPinA() >> IOPORT_PRV_PORT_OFFSET;
  int bsp_portB = DShot.get_bspPinB() >> IOPORT_PRV_PORT_OFFSET;
  int bsp_pinOffsetA = DShot.get_bspPinA() & BSP_IO_PRV_8BIT_MASK;
  int bsp_pinOffsetB = DShot.get_bspPinB() & BSP_IO_PRV_8BIT_MASK;

  R_GPT0_Type *REG_GPT;
  REG_GPT = (R_GPT0_Type *)((unsigned long)R_GPT0_BASE +
                            (unsigned long)(0x0100 * ch));

  volatile uint32_t *REG_A = &R_PFS->PORT[bsp_portA].PIN[bsp_pinOffsetA].PmnPFS;
  volatile uint32_t *REG_B = &R_PFS->PORT[bsp_portB].PIN[bsp_pinOffsetB].PmnPFS;

  message("--Registers of GPT channel %u--\n", ch);
  message("GTCR  (%p) -> 0x%08x\n", &(REG_GPT->GTCR), REG_GPT->GTCR);
  message("GTPR  (%p) -> 0x%08x(%u)\n", &(REG_GPT->GTPR), REG_GPT->GTPR,
          REG_GPT->GTPR);
  message("GTCNT (%p) -> 0x%08x(%u)\n", &(REG_GPT->GTCNT), REG_GPT->GTCNT,
          REG_GPT->GTCNT);
  message("GTPSR (%p) -> 0x%08x\n", &(REG_GPT->GTPSR), REG_GPT->GTPSR);
  message("GTSSR (%p) -> 0x%08x\n", &(REG_GPT->GTSSR), REG_GPT->GTSSR);
  message("GTCSR (%p) -> 0x%08x\n", &(REG_GPT->GTCSR), REG_GPT->GTCSR);
  message("--Registers of IOPORT--\n");
  message("Digital pin %d = PmnPFS port %d pin %d (%p) -> 0x%08x\n",
          DShot.get_pwmPinA(), bsp_portA, bsp_pinOffsetA, REG_A, *REG_A);
  message("Digital pin %d = PmnPFS port %d pin %d (%p) -> 0x%08x\n",
          DShot.get_pwmPinB(), bsp_portB, bsp_pinOffsetB, REG_B, *REG_B);
  message("--Register of ELC--\n");
  message("ELCR (%p) -> 0x%02x\n", &(R_ELC->ELCR), R_ELC->ELCR);
  for (int i = 0; i <= 18; i++) {
    if (R_ELC->ELSR[i].HA != 0) {
      message("ELSR[%d] (%p) -> 0x%04x\n", i, &(R_ELC->ELSR[i].HA),
              R_ELC->ELSR[i].HA);
    }
  }
}

bool exec_stat(char *arg) {
  HalfDuplexSerialCore::Counter_t serialCounter =
      DShot.serialCore.get_counter();
  BLHeli::counter_t blheliCounter = DShot.blHeli.get_counter();

  message("--DShot interrupts statistics--\n");
  message("tx_success: %d\n", DShot.tx_success);
  message("tx_error: %d\n", DShot.tx_error);

  message("--SerialCore interrupts statistics--\n");
  message("tx_success: %d\n", serialCounter.tx_success);
  message("rx_detect: %d\n", serialCounter.rx_detect);
  message("rx_overflow: %d\n", serialCounter.rx_overflow);
  message("rx_good_frames: %d\n", serialCounter.rx_good_frames);
  message("rx_bad_frames: %d\n", serialCounter.rx_bad_frames);
  message("spurious interrupts: %d\n", serialCounter.spurious_interrupts);

  message("--Global spurious interrupts--\n");
  message("overflow inter: %d\n", DShot.spurious_intr);

  message("--BLHeli status--\n");
  message("tx_buffer_exhausted: %d\n", blheliCounter.tx_buffer_exhausted);
  message("tx_failure: %d\n", blheliCounter.tx_failure);
  message("tx_success: %d\n", blheliCounter.tx_success);
  message("rx_buffer_exhausted: %d\n", blheliCounter.rx_buffer_exhausted);
  message("rx_timeout: %d\n", blheliCounter.rx_timeout);
  message("rx_bad_crc: %d\n", blheliCounter.rx_bad_crc);
  message("rx_success: %d\n", blheliCounter.rx_success);
  message("rx_error_verify: %d\n", blheliCounter.rx_error_verify);
  message("rx_error_command: %d\n", blheliCounter.rx_error_command);
  message("rx_error_crc: %d\n", blheliCounter.rx_error_crc);
  message("rx_error_unknown: %d\n", blheliCounter.rx_error_unknown);

  message("--MSP status--\n");
  message("received: %d, error: %d\n", Msp.received, Msp.error);
  message("--Serial Status--\n");
  message("tx_busy: %s\n", DShot.serialCore.is_tx_busy() ? "true" : "false");
  message("rx_ready: %s\n", DShot.serialCore.is_rx_ready() ? "true" : "false");
  message("--Variables--\n");
  message("auto_restart: %s\n", DShot.get_auto_restart() ? "true" : "false");
  message("auto_restart_count: %d\n", DShot.get_auto_restart_count());
  message("IFG: %u\n", DShot.get_ifg_us());
  return true;
}

bool exec_buffer(char *arg) {
  HalfDuplexSerialCore::Buffer_t serialBuffer = DShot.serialCore.get_buffer();

  message("--SerialCore buffer statistics--\n");
  message("txFIFO_Max: %d\n", serialBuffer.txFIFO_Max);
  message("txFIFO_Len: %d\n", serialBuffer.txFIFO_Len);
  message("rxFIFO_Max: %d\n", serialBuffer.rxFIFO_Max);
  message("rxFIFO_Len: %d\n", serialBuffer.rxFIFO_Len);
}

bool exec_servo_version(char *arg) {
  int verFirm, verServo;
  verFirm = SCS.readVersion(1);
  if (verFirm < 0) {
    message("failed.\n");
    return false;
  }
  verServo = SCS.readServoVersion(1);
  if (verServo < 0) {
    message("failed.\n");
    return false;
  }

  message("Firmware version: %d.%d\n", ((verFirm >> 8) & 0xFF),
          (verFirm & 0xFF));
  message("Servo version: %d.%d\n", ((verServo >> 8) & 0xFF),
          (verServo & 0xFF));
  return true;
}

bool exec_servo_baud(char *arg) {
  int baud;
  baud = SCS.readBaud(1);
  if (baud < 0) {
    message("failed.\n");
    return false;
  }

  message("read baud: %d\n", (baud & 0xFF));
  return true;
}

bool exec_servo_setbaud(char *arg) {
  int baud;
  baud = SCS.writeBaud(1, SCS0009::B38400);
  if (baud < 0) {
    message("failed.\n");
    return false;
  }

  message("written baud: %d\n", (baud & 0xff));
  return true;
}

bool exec_servo_position(char *arg) {
  int pos;

  pos = SCS.getPosition(1);
  if (pos < 0) {
    message("failed.\n");
    return false;
  }

  message("POSITION: %d\n", pos);
  return true;
}

bool exec_servo_setposition(char *arg) {
  char *endp = NULL;
  long value = strtol(arg, &endp, 0);
  if (*endp != '\0') {
    message("invalid argument \"%s\". need address value.\n", arg);
    return false;
  }
  if (value < 0x0000 || value > 0xFFFF) {
    message("invalid argument \"%s\". need address from 0x0000 to 0xFFFF.\n",
            arg);
    return false;
  }
  uint16_t pos = (uint16_t)value;

  if (SCS.setPosition(1, pos) < 0) {
    message("failed.\n");
  }

  message("POSITION: %d\n", pos);
  return true;
}

bool exec_servo_linkpwm(char *arg) {
  message("Link PWM Mode enabled. press any key to exit.\n");
  unsigned long last = millis();
  while (true) {
    unsigned long now = millis();
    if ((now - last) > 100) {
      unsigned long us = GetPWM();
      if (us > 0) {
        if (SCS.setPosition(1, (uint16_t)SCS0009::FutabaToPos(us, true, true)) <
            0) {
          message("Serial Communication Error.\n");
        }
      }
      last = now;
    }
    if (Serial.read() > 0) {
      break;
    }
  }
  message("Link PWM Mode finished.\n");

  return true;
}

bool exec_servo_config(char *arg) {
  SCS0009::config_t config = SCS.getConfig(1);

  message("---SCS0009 CONFIG---\n");
  message("ID: %d\n", config.id);
  message("BAUD: %d\n", config.baud);
  message("DELAY: %d\n", config.delay);
  message("RESPONSE_LEVEL: %d\n", config.response_level);
  message("MIN_ANGLE: %d\n", config.min_angle);
  message("MAX_ANGLE: %d\n", config.max_angle);
  message("MAX_TEMP: %d [C]\n", config.max_temp);
  message("MIN_VOLTAGE: %d.%d [V]\n", (config.min_voltage / 10),
          (config.min_voltage % 10));
  message("MAX_VOLTAGE: %d.%d [V]\n", (config.max_voltage / 10),
          (config.max_voltage % 10));

  return true;
}

bool exec_servo_status(char *arg) {
  SCS0009::status_t status = SCS.getStatus(1);

  message("---SCS0009 STATUS---\n");
  message("Target Angle: %d (%6.3f [deg])\n", status.target_angle,
          SCS0009::posToDegF(status.target_angle));
  message("Current Angle: %d (%6.3f [deg])\n", status.current_angle,
          SCS0009::posToDegF(status.current_angle));
  message("Current Velocity: %d\n", status.current_velocity);
  message("Current Load: %d.%d [%%]\n", (status.current_load / 10),
          (status.current_load % 10));
  message("Current Voltage: %d.%d [V]\n", (status.current_voltage / 10),
          (status.current_voltage % 10));
  message("Current Temp: %d [C]\n", status.current_temp);
  message("---Pin Digital 2---\n");
  unsigned long us = GetPWM();
  message("%ld (%6.3f [deg])\n", us, SCS0009::FutabaToDegF(us, true));

  return true;
}

bool exec_servo_exit(char *arg) {
  SCS.end();
  DShot.serialCore.end();
  current_table = shell_command;

  return true;
}

bool exec_bl_status(char *arg) {
  BLHeli::escStatus_t stat = DShot.blHeli.get_statusinfo();

  message("--ESC Status--\n");
  message("Protocol: %u\n", stat.protocol);
  message("good_frames: %u\n", stat.good_frames);
  message("bad_frames: %u\n", stat.bad_frames);
  message("unknown1: 0x%02x\n", stat.unknown[0]);
  message("unknown2: 0x%02x\n", stat.unknown[1]);
  message("unknown3: 0x%02x\n", stat.unknown[2]);
  message("unknown4: 0x%04x\n", stat.unknown2);
}

bool exec_bl_bootinfo(char *arg) {
  BLHeli::bootInfo_t info = DShot.blHeli.get_bootinfo();
  char rstr[5];

  char *p = (char *)&info.Revision;
  for (int i = 0; i < 4; i++) {
    if (isprint(p[i])) {
      rstr[3 - i] = p[i];
    } else {
      rstr[i] = ' ';
    }
  }
  rstr[4] = '\0';

  const char *sCPU;
  switch (info.CPUType) {
    case BLHeli::ATMEL:
      sCPU = "Atmel Corporation";
      break;
    case BLHeli::SILAB:
      sCPU = "Silicon Laboratories";
      break;
    case BLHeli::ARM:
      sCPU = "Arm Limited";
      break;
    default:
      sCPU = "Unknown";
      break;
  }

  message("BootInfo:\n");
  message("Revision: 0x%08x (%s)\n", info.Revision, rstr);
  message("Signature: 0x%04x (%s)\n", info.Signature, sCPU);
  message("Boot Version: %u\n", info.Version);
  message("Boot Pages: %u\n", info.Pages);

  return true;
}

bool exec_bl_firminfo(char *arg) {
  BLHeli::firmInfo_t info = DShot.blHeli.get_firminfo();

  if (!info.present) {
    message("No firmware information retrieved.\n");
    return false;
  }

  message("FirmInfo:\n");
  message("Revision %d.%d\n", info.mainRevision, info.subRevision);
  message("EEPROM Layout Revision: %d\n", info.eepromLayout);
  message("Layout: \"%s\"\n", info.layoutTag);
  message("MCU: \"%s\"\n", info.mcuTag);
  message("Name: \"%s\"\n", info.nameTag);
  return true;
}

bool exec_bl_keepalive(char *arg) {
  message("Sending keepalive...");
  bool result = DShot.blHeli.keepAlive();
  if (result) {
    message("success.\n");
  } else {
    message("failure.\n");
  }

  return result;
}

bool exec_bl_setaddress(char *arg) {
  if (arg == NULL) {
    message("missing argument.\n");
    return false;
  }

  char *endp = NULL;
  long value = strtol(arg, &endp, 0);
  if (*endp != '\0') {
    message("invalid argument \"%s\". need address value.\n", arg);
    return false;
  }
  if (value < 0x0000 || value > 0xFFFF) {
    message("invalid argument \"%s\". need address from 0x0000 to 0xFFFF.\n",
            arg);
    return false;
  }
  message("send set address command...");
  bool result = DShot.blHeli.setAddress((uint16_t)value);
  if (result) {
    last_address = value;
    message("success.\n");
  } else {
    message("failure.\n");
  }
  return result;
}

bool exec_bl_readflash(char *arg) {
  uint8_t buf[128];

  message("send read flash command...");
  uint8_t code =
      DShot.blHeli.readDataRaw(BLHeli::READ_FLASH_SIL, buf, sizeof(buf));
  if (code != BLHeli::SUCCESS) {
    message("failure(0x%02x: %s).\n", code, bl_strerror(code));
    return false;
  }
  message("success.\n");

  hexdump(buf, sizeof(buf));

  return true;
}

bool exec_bl_readeeprom(char *arg) {
  uint8_t buf[128];

  message("send read eeprom command...");
  uint8_t code =
      DShot.blHeli.readDataRaw(BLHeli::READ_EEPROM, buf, sizeof(buf));
  if (code != BLHeli::SUCCESS) {
    message("failure(0x%02x: %s).\n", code, bl_strerror(code));
    return false;
  }
  message("success.\n");

  hexdump(buf, sizeof(buf));

  return true;
}
bool exec_bl_readsram(char *arg) {
  uint8_t buf[128];

  message("send read sram command...");
  uint8_t code = DShot.blHeli.readDataRaw(BLHeli::READ_SRAM, buf, sizeof(buf));
  if (code != BLHeli::SUCCESS) {
    message("failure(0x%02x: %s).\n", code, bl_strerror(code));
    return false;
  }
  message("success.\n");

  hexdump(buf, sizeof(buf));

  return true;
}

bool exec_bl_restart(char *arg) {
  message("Restarting bootloader...");
  bool result = DShot.blHeli.restart();
  if (result) {
    message("success.\n");
  } else {
    message("failure.\n");
  }

  return result;
}

bool exec_bl_open(char *arg) {
  uint8_t bootInfo[9];

  message("hello sequence sent to BLHeli...");
  if (!DShot.blHeli.sendSignature()) {
    message("failure.\n");
  }
  message("done.\n");
  return true;
}

bool exec_bl_write(char *arg) {
  char *argp = arg;
  int n = 0;

  if (arg == NULL) {
    message("need data to write.\n");
    return false;
  }

  while (*argp != '\0') {
    DShot.serialCore.write((uint8_t)*argp);
    argp++;
    n++;
  }

  message("written %d bytes.\n", n);
  return true;
}

bool exec_bl_exit(char *arg) {
  message("Finish bootloader session...");
  DShot.bootloader_exit();
  message("done.\n");
  current_table = shell_command;

  return true;
}

bool exec_arm(char *arg) {
  DShot.arm();
  message("arm sequence triggered.\n");
  return true;
}

bool exec_reset(char *arg) {
  message("Initializing ESC communication...\n");
  DShot.reset();
  message("ESC communication initialized.\n");

  return true;
}

bool exec_suspend(char *arg) {
  DShot.suspend();
  message("DShot protocol suspended.\n");

  return true;
}

bool exec_disarm(char *arg) {
  DShot.set_command(CHANNEL_B, DShotR4::DSHOT_CMD_DISARM);
  DShot.transmit();
  message("disarmed.\n");

  return true;
}

bool exec_throttle(char *arg) {
  if (arg == NULL || *arg == '\0') {
    message("missing argument. need throttle level from 48 to 2047.\n");
    return false;
  }

  char *endp = NULL;
  long value = strtol(arg, &endp, 10);

  if (*endp != '\0') {
    message("invalid argument \"%s\". need throttle level from 48 to 2047.\n",
            arg);
    return false;
  }

  if (value < 48 || value > 2047) {
    message("argument is out of range. need throttle level from 48 to 2047.\n",
            arg);
    return false;
  }

  DShot.set_throttle(CHANNEL_B, (uint16_t)value);
  message("set throttle to %d. (need \"commit\" command to take effect.)\n",
          value);
  return true;
}

bool exec_commit(char *arg) {
  DShot.transmit();
  message("commit current throttle value.\n");
  return true;
}

bool exec_bootloader(char *arg) {
  message("Entering BLHeli bootloader mode...");
  if (!DShot.bootloader_enter()) {
    message("failure\n");
  } else {
    message("done\n");
  }
  current_table = bootloader_command;
  return true;
}

bool exec_servo(char *arg) {
  message("Entering Servo tester mode...");
  DShot.serialCore.begin(CHANNEL_B, 0, 1000 * 1000, false);
  SCS.begin();
  SCS.writeBaud(1, SCS0009::B38400);
  SCS.writeBaud(1, SCS0009::B38400);
  SCS.end();
  DShot.serialCore.end();
  DShot.serialCore.begin(CHANNEL_B, 0, 38400, false);
  SCS.begin();
  message("done\n");
  current_table = serial_servo_command;
}

bool exec_help(char *arg) {
  show_command_list();

  return true;
}

bool show_command_list() {
  struct shell_command_table *tp = current_table;

  message("commands:");
  while (tp->name != NULL) {
    message(" %s", tp->name);
    tp++;
  }
  message("\n");

  return true;
}

bool shell(char *command, char *arg) {
  struct shell_command_table *tp = current_table;

  if (strcmp(command, "?") == 0) {
    show_command_list();
    return true;
  }

  while (tp->name != NULL) {
    if (strcmp(command, tp->name) == 0) {
      return tp->exec(arg);
    }
    tp++;
  }
  message("unknown command: %s\n", command);
  show_command_list();

  return false;
}
