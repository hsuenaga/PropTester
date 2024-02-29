struct shell_command_table {
  const char *name;
  bool (*exec)(char *);
};

struct shell_command_table shell_command[] = {
  {"reg", exec_reg},
  {"stat", exec_stat},
  {"arm", exec_arm},
  {"reset", exec_reset},
  {"suspend", exec_suspend},
  {"disarm", exec_disarm},
  {"throttle", exec_throttle},
  {"commit", exec_commit},
  {"bootloader", exec_bootloader},
  {"help", exec_help},
  {NULL, NULL}
};

struct shell_command_table bootloader_command[] = {
  {"reg", exec_reg},
  {"stat", exec_stat},
  {"buffer", exec_bl_buffer},
  {"open", exec_bl_open},
  {"write", exec_bl_write},
  {"exit", exec_bl_exit},  
  {NULL, NULL}
};

struct shell_command_table *current_table = shell_command;

bool
exec_reg(char *arg)
{
  uint8_t ch = DShot.get_channel();
  int bsp_portA = DShot.get_bspPinA() >> IOPORT_PRV_PORT_OFFSET;
  int bsp_portB = DShot.get_bspPinB() >> IOPORT_PRV_PORT_OFFSET;
  int bsp_pinOffsetA = DShot.get_bspPinA() & BSP_IO_PRV_8BIT_MASK;
  int bsp_pinOffsetB = DShot.get_bspPinB() & BSP_IO_PRV_8BIT_MASK;

  R_GPT0_Type *REG_GPT;
  REG_GPT = (R_GPT0_Type *)((unsigned long)R_GPT0_BASE + (unsigned long)(0x0100 * ch));

  volatile uint32_t *REG_A = &R_PFS->PORT[bsp_portA].PIN[bsp_pinOffsetA].PmnPFS;
  volatile uint32_t *REG_B = &R_PFS->PORT[bsp_portB].PIN[bsp_pinOffsetB].PmnPFS;

  message("--Registers of GPT channel %u--\n", ch);
  message("GTCR  (%p) -> 0x%08x\n", &(REG_GPT->GTCR), REG_GPT->GTCR);
  message("GTPR  (%p) -> 0x%08x(%u)\n", &(REG_GPT->GTPR), REG_GPT->GTPR, REG_GPT->GTPR);
  message("GTCNT (%p) -> 0x%08x(%u)\n", &(REG_GPT->GTCNT), REG_GPT->GTCNT, REG_GPT->GTCNT);
  message("GTPSR (%p) -> 0x%08x\n", &(REG_GPT->GTPSR), REG_GPT->GTPSR);
  message("GTSSR (%p) -> 0x%08x\n", &(REG_GPT->GTSSR), REG_GPT->GTSSR);
  message("GTCSR (%p) -> 0x%08x\n", &(REG_GPT->GTCSR), REG_GPT->GTCSR);
  message("--Registers of IOPORT--\n");
  message("Digital pin %d = PmnPFS port %d pin %d (%p) -> 0x%08x\n", DShot.get_pwmPinA(), bsp_portA, bsp_pinOffsetA, REG_A, *REG_A);
  message("Digital pin %d = PmnPFS port %d pin %d (%p) -> 0x%08x\n", DShot.get_pwmPinB(), bsp_portB, bsp_pinOffsetB, REG_B, *REG_B);
  message("--Register of ELC--\n");
  message("ELCR (%p) -> 0x%02x\n", &(R_ELC->ELCR), R_ELC->ELCR);
  for (int i = 0; i <= 18; i++) {
    if (R_ELC->ELSR[i].HA != 0) {
      message("ELSR[%d] (%p) -> 0x%04x\n", i, &(R_ELC->ELSR[i].HA), R_ELC->ELSR[i].HA);
    }
  }
}

bool
exec_stat(char *arg)
{
  message("--Interrupts statistics--\n");
  message("tx_success: %d\n", DShot.tx_success);
  message("tx_error: %d\n", DShot.tx_error);
  message("tx_serial_success: %d\n", DShot.tx_serial_success);
  message("rx_serial_detect: %d\n", DShot.rx_serial_detect);
  message("rx_serial_good_frames: %d\n", DShot.rx_serial_good_frames);
  message("rx_serial_bad_frames: %d\n", DShot.rx_serial_bad_frames);
  message("sprious interrupts: %d\n", DShot.suprious_intr);
  message("--MSP status--\n");
  message("received: %d, error: %d\n", Msp.received, Msp.error);
  message("--Variables--\n");
  message("auto_restart: %s\n", DShot.get_auto_restart() ? "true" : "false");
  message("auto_restart_count: %d\n", DShot.get_auto_restart_count());
  message("IFG: %u\n", DShot.get_ifg_us());
  return true;
}

bool
exec_bl_buffer(char *arg)
{
  uint8_t buf[10];
  size_t len = sizeof(buf);

  message("CHANNEL_A:");
  DShot.get_bl_rx_raw_buff(CHANNEL_A, buf, &len);
  for (int i = 0; i < len; i++) {
    message(" %02x", buf[i]);
  }
  message("\n");

  message("CHANNEL_B:");
  DShot.get_bl_rx_raw_buff(CHANNEL_B, buf, &len);
  for (int i = 0; i < len; i++) {
    message(" %02x", buf[i]);
  }
  message("\n");

  message("DEBUG    :");
  DShot.get_bl_rx_debug_buff(buf, &len);
  for (int i = 0; i < len; i++) {
    message(" %02x", buf[i]);
  }
  message("\n");

  message("CURRENT A: %02x\n", R_PFS->PORT[DShot.get_bspPinA() >> IOPORT_PRV_PORT_OFFSET].PIN[DShot.get_bspPinA() & BSP_IO_PRV_8BIT_MASK].PmnPFS_BY);
  message("CURRENT B: %02x\n", R_PFS->PORT[DShot.get_bspPinB() >> IOPORT_PRV_PORT_OFFSET].PIN[DShot.get_bspPinB() & BSP_IO_PRV_8BIT_MASK].PmnPFS_BY);
}

bool
exec_bl_open(char *arg)
{
  uint8_t bootInfo[9];
  long start = millis();

  DShot.bl_open();
  message("hello sequence sent to BLHeli...");
  while (DShot.bl_peek() < sizeof(bootInfo)) {
    yield();
    if ((millis() - start) > 1000) {
      message("timeout.\n");
      DShot.bl_flush();
      return false;
    }
  }
  message("done.\n");
  message("BootInfo:");
  for (int i = 0; i < sizeof(bootInfo); i++) {
    bootInfo[i] = (uint8_t)DShot.bl_read();
    message(" %02x", bootInfo[i]);
  }
    DShot.bl_flush();
  message("\n");
  // BOOT_MSG
  message("BootMessage(Revision): %c%c%c%c\n", bootInfo[0], bootInfo[1], bootInfo[2], bootInfo[3]);
  // SIGNATURE_001
  message("Signature.1: %02x\n", bootInfo[4]);
  // SIGNATURE_002
  message("Signature.2: %02x\n", bootInfo[5]);
  // BOOT_VERSION
  message("Boot Version: %u\n", bootInfo[6]);
  // BOOT_PAGES
  message("Boot Pages: %u\n", bootInfo[7]);
  // COMMAND_STATUS
  message("Command STATUS: 0x%0x\n", bootInfo[8]);


  return true;
}

bool
exec_bl_write(char *arg)
{
  char *argp = arg;
  int n = 0;

  if (arg == NULL) {
    message("need data to write.\n");
    return false;
  }

  while (*argp != '\0') {
    DShot.bl_write((uint8_t)*argp);
    argp++;
    n++;
  }

  message("written %d bytes.\n", n);
  return true;
}

bool
exec_bl_exit(char *arg)
{
  message("Finish bootloader session...");
  DShot.bl_exit();
  message("done.\n");
  current_table = shell_command;

  return true;
}

bool
exec_arm(char *arg)
{
  DShot.arm();
  message("arm sequence triggered.\n");
  return true;
}

bool
exec_reset(char *arg)
{
  message("Initalizing ESC communication...\n");
  DShot.reset();
  message("ESC communication initialized.\n");

  return true;
}

bool
exec_suspend(char *arg)
{
    DShot.suspend();
    message("DShot protocol suspended.\n");

    return true;
}

bool
exec_disarm(char *arg)
{
  DShot.set_command(CHANNEL_B, DShotR4::DSHOT_CMD_DISARM);
  DShot.transmit();
  message("disarmed.\n");

  return true;
}

bool
exec_throttle(char *arg)
{
  if (arg == NULL || *arg == '\0') {
    message("missing argument. need throttle level from 48 to 2047.\n");
    return false;
  }

  char *endp = NULL;
  long value = strtol(arg, &endp, 10);

  if (*endp != '\0') {
    message("invalid argument \"%s\". need throttle level from 48 to 2047.\n", arg);
    return false;
  }

  if (value < 48 || value > 2047) {
    message("argument is out of range. need throttle level from 48 to 2047.\n", arg);
    return false;
  }

  DShot.set_throttle(CHANNEL_B, (uint16_t) value);
  message("set throttle to %d. (need \"commit\" command to take effect.)\n", value);
  return true;
}

bool
exec_commit(char *arg)
{
  DShot.transmit();
  message("commit current throttle value.\n");
  return true;
}

bool
exec_bootloader(char *arg)
{
  message("Entering BLHeli bootloader mode...");
  DShot.bl_enter();
  message("done\n");
  current_table = bootloader_command;
  return true;
}

bool
exec_help(char *arg)
{
  show_command_list();

  return true;
}

bool
show_command_list()
{
  struct shell_command_table *tp = current_table;

  message("commands:");
  while (tp->name != NULL) {
    message(" %s", tp->name);
    tp++;
  }
  message("\n");

  return true;
}

bool
shell(char *command, char *arg)
{
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
