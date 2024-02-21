char consoleBuffer[256];
char *consoleBufferp = consoleBuffer;
const char *consoleEndp = consoleBufferp + sizeof(consoleBuffer);
bool lineCompleted = false;
bool consoleEcho = true;

bool
consoleReceive(void)
{
  if (Serial.available() <= 0 || lineCompleted) {
    return false;
  }
  int byte = Serial.read();
  switch (byte) {
    case 0x0d:
      // ignore 0x0d
      break;
    case 0x0a:
      lineCompleted = true;
      Serial.println("");
      break;
    default:
      if (!isprint(byte)) {
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
  if (lineCompleted) {
    return true;
  }
  return false;
}

bool
consoleRead(char *dst, size_t dstlen)
{
  size_t srclen = consoleBufferp - consoleBuffer;

  if (!lineCompleted) {
    return false;
  }

  if (srclen > (dstlen - 1)) {
    memcpy(dst, consoleBuffer, (dstlen - 1));
    dst[dstlen - 1] = '\0';
  }
  else {
    memcpy(dst, consoleBuffer, srclen);
    dst[srclen] = '\0';
  }
  consoleBufferp = consoleBuffer;
  lineCompleted = false;
  return true;
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

  if (*command != '\0') {
    shell(command, arg);
  }

  return true;
}