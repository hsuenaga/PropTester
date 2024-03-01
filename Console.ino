char consoleBuffer[256];
char *consoleBufferp = consoleBuffer;
const char *consoleEndp = consoleBufferp + sizeof(consoleBuffer) - 1;
bool lineCompleted = false;
bool skipLF = false;
bool consoleEcho = true;
enum consoleState_t
{
  DEFAULT = 0,
  MSP
};
enum consoleState_t consoleState = DEFAULT;

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

bool
consoleReceive(void)
{
  if (Serial.available() <= 0 || lineCompleted) {
    return false;
  }
  int byte = Serial.read();

  // Intercept MSP/BLIHELI frame.
  if (consoleBufferp == consoleBuffer && consoleState != MSP && byte == '$') {
    consoleState = MSP;
  }
  if (consoleState == MSP) {
    if (Msp.receive(byte)) {
      return true;
    }
    consoleState = DEFAULT;
  }

  switch (byte) {
    case 0x0d:
      lineCompleted = true;
      skipLF = true;
      // ignore next 0x0a
      break;
    case 0x0a:
      if (skipLF) {
        skipLF = false;
        break;
      }
      lineCompleted = true;
      break;
    default:
      skipLF = false;
      if (!isprint(byte))
      {
        break;
      }
      if (consoleBufferp < consoleEndp) {
        *consoleBufferp++ = (char)byte;
        if (consoleEcho) {
          Serial.write(byte);
        }
      }
      break;
  }
  if (!lineCompleted) {
    return false;
  }
  Serial.println("");

  *consoleBufferp = '\0';
  consoleBufferp = consoleBuffer;
  lineCompleted = false;
  return consoleParse(consoleBuffer);
}

bool
consoleParse(char *rcvbuf)
{
  char *command = rcvbuf;
  char *arg = NULL;
  char *bufp = rcvbuf;

  while (*bufp != '\0') {
    switch (*bufp) {
      case ' ':
        *bufp = '\0';
        bufp++;
        arg = (*bufp == '\0') ? NULL : bufp;
        break;
      default:
        bufp++;
        break;
    }
  }

  if (*command == '\0') {
    return false;
  }

  return shell(command, arg);
}
