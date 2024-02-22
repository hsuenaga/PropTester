struct shell_command_table {
  const char *name;
  bool (*exec)(char *);
} shell_command[] = {
  {"stat", exec_stat},
  {"arm", exec_arm},
  {"reset", exec_reset},
  {"suspend", exec_suspend},
  {"disarm", exec_disarm},
  {"throttle", exec_throttle},
  {"commit", exec_commit},
  {"help", exec_help},
  {NULL, NULL}
};

bool
exec_stat(char *arg)
{
  message("tx_success: %d, tx_error: %d\n", DShot.tx_success, DShot.tx_error);
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
exec_help(char *arg)
{
  show_command_list();

  return true;
}

bool
show_command_list()
{
  struct shell_command_table *tp = shell_command;

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
  struct shell_command_table *tp = shell_command;

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
