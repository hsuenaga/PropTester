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
  {"write", exec_bl_write},
  {"exit", exec_bl_exit},
  {NULL, NULL}
};

struct shell_command_table *current_table = shell_command;

bool
exec_reg(char *arg)
{
  R_GPT0_Type *gpt_reg;
  int i = 4;
  gpt_reg = (R_GPT0_Type *)((unsigned long)R_GPT0_BASE + (unsigned long)(0x0100 * i));

  message("gpt_reg->GTCR  (%p) -> 0x%x\n", &(gpt_reg->GTCR), gpt_reg->GTCR);
  message("gpt_reg->GTCNT (%p) -> 0x%x\n", &(gpt_reg->GTCNT), gpt_reg->GTCNT);
  message("gpt_reg->GTPSR (%p) -> 0x%x\n", &(gpt_reg->GTPSR), gpt_reg->GTPSR);
}

bool
exec_stat(char *arg)
{
  message("--DShot status--\n");
  message("tx_success: %d\n", DShot.tx_success);
  message("tx_error: %d\n", DShot.tx_error);
  message("tx_serial_success: %d\n", DShot.tx_serial_success);
  message("--MSP status--\n");
  message("received: %d, error: %d\n", Msp.received, Msp.error);
  message("--Variables--\n");
  message("auto_restart: %s\n", DShot.get_auto_restart() ? "true" : "false");
  message("auto_restart_count: %d\n", DShot.get_auto_restart_count());
  message("IFG: %u\n", DShot.get_ifg_us());
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

  message("written %d bytes.", n);
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
