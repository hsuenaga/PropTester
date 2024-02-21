#include <Arduino.h>
#include <stdarg.h>

#include "HX711R4.h"
#include "DShotR4.h"

//---------------------------------------------------//
// ピンの設定
//---------------------------------------------------//
#define pin_dout  8
#define pin_slk   9

static HX711 HX711;
static DShotR4 DShot;

#undef HX711_TEST
#undef DSHOT_TEST

void setup() {
  Serial.begin(115200);

#ifdef HX711_TEST
  HX711.init(pin_dout, pin_slk);
  HX711.reset();
  HX711.tare();
#endif

  DShot.init(DShotR4::DSHOT300);
  DShot.reset();
}

void
AE_HX711_Print()
{
  char S1[80], Stmp[16];
  double value = HX711.getGram();
  long snapValue;

  snapValue = HX711.acquire();
  dtostrf(value, 5, 3, Stmp);

  snprintf(S1, sizeof(S1), "%s [g] (0x%08x)", Stmp, snapValue);
  Serial.println(S1);
}

int
message(const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  char strbuf[256];
  int ret;

  strbuf[0] = '\0';
  ret = vsnprintf(strbuf, sizeof(strbuf), fmt, ap);
  va_end(ap);

  Serial.write(strbuf);
  return ret;
}

void
shell_help(void)
{
  message("commands: stat, arm, disarm, reset, suspend, throttle, commit, help\n");
}

void
shell(char *command, char *arg)
{
  if (strcmp(command, "stat") == 0) {
    message("tx_success: %d, tx_error: %d\n", DShot.tx_success, DShot.tx_error);
  }
  else if (strcmp(command, "arm") == 0) {
    DShot.arm();
    message("arm sequence triggered.\n");
  }
  else if (strcmp(command, "reset") == 0) {
    message("Initalizing ESC communication...\n");
    DShot.reset();
    message("ESC communication initialized.\n");
  }
  else if (strcmp(command, "suspend") == 0) {
    DShot.suspend();
    message("DShot protocol suspended.\n");
  }
  else if (strcmp(command, "disarm") == 0) {
    DShot.set_command(CHANNEL_B, DShotR4::DSHOT_CMD_DISARM);
    message("disarmed.\n");
  }
  else if (strcmp(command, "throttle") == 0) {
    if (arg == NULL || *arg == '\0') {
      message("missing argument. need throttle level from 48 to 2047.\n");      
    }
    else {
      char *endp = NULL;
      long value = strtol(arg, &endp, 10);
      if (*endp != '\0') {
        message("invalid argument \"%s\". need throttle level from 48 to 2047.\n", arg);
      }
      else if (value < 48 || value > 2047) {
        message("argument is out of range. need throttle level from 48 to 2047.\n", arg);
      } else {
        DShot.set_throttle(CHANNEL_B, (uint16_t) value);
        message("set throttle to %d. (need \"commit\" command to take effect.)\n", value);
      }
    }
  }
  else if (strcmp(command, "commit") == 0) {
    DShot.transmit();
    message("commit current throttle value.\n");
  }
  else if (strcmp(command, "help") == 0 || strcmp(command, "?") == 0) {
    shell_help();
  }
  else {
    message("unknown command: %s\n", command);
    shell_help();
  }
}

void loop() 
{ 
  static uint32_t count = 0;
  static unsigned long last_time = 0;
  static uint16_t dshot_throttle = 0;

  unsigned long time = micros();
  unsigned long elapsed = time - last_time;

  if (elapsed > 250) {
    last_time = time;
  }

#ifdef HX711_TEST
  if (HX711.isDataReady()) 
  {
    HX711.acquire();
//    AE_HX711_Print();
  }
#endif

  if (consoleReceive()) {
    char rcvbuf[256];

    consoleRead(rcvbuf, sizeof(rcvbuf));
    consoleParse(rcvbuf);
  }
}