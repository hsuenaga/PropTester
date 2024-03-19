#include "BLHeli.h"

uint16_t BLHeli::crcCompute(const uint8_t *buf, size_t len) {
  const uint16_t poly = 0xA001;
  uint16_t crc = 0;

  while (len--) {
    uint8_t xb = *buf++;

    for (int i = 0; i < 8; i++) {
      if (((xb & 0x01) ^ (crc & 0x0001)) != 0) {
        crc >>= 1;
        crc = crc ^ poly;
      } else {
        crc >>= 1;
      }
      xb >>= 1;
    }
  }

  return crc;
}

bool BLHeli::send(const uint8_t *buf, size_t len) {
  if (len > sizeof(txrxBuf)) {
    counter.tx_buffer_exhausted++;
    return false;
  }
  memcpy(txrxBuf, buf, len);
  uint16_t crc = crcCompute(buf, len);
  txrxBuf[len] = crc & 0xFF;
  txrxBuf[len + 1] = (crc >> 8) & 0xFF;

  size_t n = port.write(&txrxBuf[0], len + 2);
  if (n != (len + 2)) {
    counter.tx_failure++;
    return false;
  }
  counter.tx_success++;
  return true;
}

uint8_t BLHeli::recv(uint8_t *buf, size_t len, bool hasCRC) {
  int recvLen = hasCRC ? len + 3 : len + 1;
  if (recvLen > sizeof(txrxBuf)) {
    counter.rx_buffer_exhausted++;
    return NONE;
  }

  uint8_t code;
  int n = port.readBytes(&txrxBuf[0], recvLen);
  if (n != recvLen && n != 1) {
    counter.rx_timeout++;
    return NONE;
  } else if (n == 1) {
    // detect single byte response.
    // note: error code may be returned without CRC.
    code = txrxBuf[0];
  } else {
    // detect multibyte response.
    if (hasCRC) {
      // multibyte response with CRC
      uint16_t crc = crcCompute(&txrxBuf[0], len);
      if (txrxBuf[len] != (crc & 0xFF)) {
        counter.rx_bad_crc++;
        return NONE;
      }
      if (txrxBuf[len + 1] != ((crc >> 8) & 0xFF)) {
        counter.rx_bad_crc++;
        return NONE;
      }
    }
    code = txrxBuf[recvLen - 1];
  }

  code = observeResultCode(code);
  if (code == SUCCESS && buf != NULL && len > 0) {
    memcpy(buf, &txrxBuf[0], len);
  }
  return code;
}

uint8_t BLHeli::recvAck() { return recv(NULL, 0, false); }

uint8_t BLHeli::observeResultCode(uint8_t code) {
  switch (code) {
    case SUCCESS:
      counter.rx_success++;
      break;
    case ERRORVERIFY:
      counter.rx_error_verify++;
      break;
    case ERRORCOMMAND:
      counter.rx_error_command++;
      break;
    case ERRORCRC:
      counter.rx_error_crc++;
      break;
    case NONE:
    default:
      counter.rx_error_unknown++;
      break;
  }

  return code;
}

void BLHeli::parseBootInfo(uint8_t (&buf)[8]) {
  bootInfo.Revision = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
  bootInfo.Signature = (buf[4] << 8) | buf[5];
  bootInfo.Version = buf[6];
  bootInfo.Pages = buf[7];
  memset(&address, 0, sizeof(address));

  switch (bootInfo.Signature) {
    case 0x9307:
    case 0x930A:
    case 0x930F:
    case 0x940B:
      bootInfo.CPUType = ATMEL;
      break;
    case 0xF310:
    case 0xF330:
    case 0xF410:
    case 0xF390:
    case 0xF850:
      bootInfo.CPUType = SILAB;
      break;
    case 0xE8B1:
    case 0xE8B2:
      address.versionTag = 0x1A00;
      address.versionTagLen = 3;
      address.layoutTag = 0x1A40;
      address.layoutTagLen = 16;
      address.mcuTag = 0x1A50;
      address.mcuTagLen = 16;
      address.nameTag = 0x1A60;
      address.nameTagLen = 16;
      bootInfo.CPUType = SILAB;
      break;
    case 0xE8B5:
      address.versionTag = 0x3000;
      address.versionTagLen = 3;
      address.layoutTag = 0x3040;
      address.layoutTagLen = 16;
      address.mcuTag = 0x3050;
      address.mcuTagLen = 16;
      address.nameTag = 0x3060;
      address.nameTagLen = 16;
      bootInfo.CPUType = SILAB;
      break;
    default:
      if (buf[4] == 0x06 && buf[5] > 0x00 && buf[5] < 0x90) {
        bootInfo.CPUType = ARM;
      } else {
        bootInfo.CPUType = UNKNOWN;
      }
      break;
  }
}

BLHeli::BLHeli(Stream &s) : port(s) {}

BLHeli::~BLHeli() {}

bool BLHeli::begin() { return sendSignature(); }

bool BLHeli::end() { return restart(); }

bool BLHeli::sendSignature() {
  uint8_t buf[8];

  memset(&bootInfo, 0, sizeof(bootInfo));
  memset(&firmInfo, 0, sizeof(firmInfo));
  restart();
  restart();
  send(blheli_signature, sizeof(blheli_signature));

  uint8_t code = recv(buf, sizeof(buf), false);
  if (code != SUCCESS) {
    return false;
  }
  parseBootInfo(buf);

  (void)readFirmInfo();
  return true;
}

bool BLHeli::keepAlive() {
  // Actually, command KEEP_ALIVE is not implemented on ESCs.
  // ERRORCOMMAND will be returned if ESC works fine.
  uint8_t cmd[] = {KEEP_ALIVE, 0};
  if (send(cmd, 2) == false) {
    return false;
  }

  uint8_t code = recvAck();
  if (code == ERRORCOMMAND) {
    return true;
  }

  return false;
}

bool BLHeli::restart() {
  uint8_t cmd[] = {RUN, 0};
  if (send(cmd, sizeof(cmd)) == false) {
    return false;
  }

  return true;
}

uint8_t BLHeli::setAddress(uint16_t addr) {
  if (addr == 0xFFFF) {
    return NONE;
  }

  uint8_t cmd[4] = {SET_ADDRESS, 0, ((addr >> 8) & 0xFF), (addr & 0xFF)};

  if (send(cmd, sizeof(cmd)) == false) {
    return NONE;
  }
  uint8_t code = recvAck();
  if (code != SUCCESS) {
    return NONE;
  }
  address_present = true;
  return code;
}

uint8_t BLHeli::setBuffer(const uint8_t *buf, uint16_t len) {
  uint8_t cmd[4] = {SET_BUFFER, 0, (uint8_t)((len >> 8) & 0xFF),
                    (uint8_t)(len & 0xFF)};

  if (send(cmd, sizeof(cmd)) == false) {
    return NONE;
  }
  if (recvAck() != SUCCESS) {
    return NONE;
  }

  if (send(buf, len) == false) {
    return NONE;
  }
  uint8_t code = recvAck();
  if (code != SUCCESS) {
    return NONE;
  }
  buffer_present = true;
  return code;
}

uint8_t BLHeli::readDataRaw(uint8_t type, uint8_t *buf, uint16_t len) {
  if (!address_present) {
    return NONE;
  }
  if (len > 256) {
    return NONE;
  }
  switch (type) {
    case READ_FLASH_SIL:
    case READ_FLASH_ATM:
      if (bootInfo.CPUType == ATMEL) {
        type = READ_FLASH_ATM;
      } else {
        type = READ_FLASH_SIL;
      }
      break;
    case READ_EEPROM:
    case READ_SRAM:
      break;
    default:
      return NONE;
  }

  address_present = false;
  uint8_t cmd[2] = {type, (uint8_t)(len == 256 ? 0 : len)};
  if (send(cmd, sizeof(cmd)) == false) {
    return NONE;
  }

  return recv(buf, len);
}

uint8_t BLHeli::writeDataRaw(uint8_t type) {
  if (!address_present || !buffer_present) {
    return NONE;
  }
  switch (type) {
    case PROG_FLASH:
    case PROG_EEPROM:
      break;
    default:
      return NONE;
  }

  address_present = false;
  buffer_present = false;
  uint8_t cmd[2] = {type, 0x01};
  if (send(cmd, sizeof(cmd)) == false) {
    return NONE;
  }

  port.setTimeout(3000);
  uint8_t code = recvAck();
  port.setTimeout(1000);

  return code;
}

uint8_t BLHeli::verifyDataRaw(uint8_t type) {
  if (!address_present || !buffer_present) {
    return NONE;
  }
  switch (type) {
    case VERIFY_FLASH:
    case VERIFY_FLASH_ARM:
      if (bootInfo.CPUType == ATMEL) {
        type = VERIFY_FLASH;
      } else {
        type = VERIFY_FLASH_ARM;
      }
      break;
    default:
      return NONE;
  }

  address_present = false;
  buffer_present = false;
  uint8_t cmd[2] = {type, 0x01};
  if (send(cmd, sizeof(cmd)) == false) {
    return NONE;
  }

  return recvAck();
}

uint8_t BLHeli::pageEraseRaw() {
  if (!address_present) {
    return NONE;
  }

  address_present = false;
  uint8_t cmd[2] = {ERASE_FLASH, 0x01};
  if (send(cmd, sizeof(cmd)) == false) {
    return NONE;
  }

  port.setTimeout(3000);
  uint8_t code = recvAck();
  port.setTimeout(1000);

  return code;
}

bool BLHeli::readData(uint8_t type, uint16_t addr, uint8_t *buf, uint16_t len) {
  if (setAddress(addr) != SUCCESS) {
    return false;
  }
  if (readDataRaw(type, buf, len) != SUCCESS) {
    address_present = false;
    return false;
  }
  return true;
}

bool BLHeli::writeData(uint8_t type, uint16_t addr, const uint8_t *buf,
                       uint16_t len) {
  if (setAddress(addr) != SUCCESS) {
    return false;
  }
  if (setBuffer(buf, len) != SUCCESS) {
    address_present = false;
    return false;
  }
  if (writeDataRaw(type) != SUCCESS) {
    address_present = false;
    buffer_present = false;
    return false;
  }
  return true;
}

bool BLHeli::verifyData(uint8_t type, uint16_t addr, const uint8_t *buf,
                        uint16_t len) {
  if (setAddress(addr) != SUCCESS) {
    return false;
  }
  if (setBuffer(buf, len) != SUCCESS) {
    address_present = false;
    return false;
  }
  if (verifyDataRaw(type) != SUCCESS) {
    address_present = false;
    buffer_present = false;
    return false;
  }
  return true;
}

bool BLHeli::readFlash(uint16_t addr, uint8_t *buf, uint16_t len) {
  return readData(READ_FLASH_SIL, addr, buf, len);
}

bool BLHeli::writeFlash(uint16_t addr, const uint8_t *buf, uint16_t len) {
  if (writeData(PROG_FLASH, addr, buf, len) == false) {
    return false;
  }
  return verifyData(VERIFY_FLASH, addr, buf, len);
}

bool BLHeli::readEEPROM(uint16_t addr, uint8_t *buf, uint16_t len) {
  return readData(READ_EEPROM, addr, buf, len);
}

bool BLHeli::writeEEPROM(uint16_t addr, const uint8_t *buf, uint16_t len) {
  if (writeData(PROG_EEPROM, addr, buf, len) == false) {
    return false;
  }
  // need read & verify ???

  return true;
}

bool BLHeli::readSRAM(uint16_t addr, uint8_t *buf, uint16_t len) {
  return readData(READ_SRAM, addr, buf, len);
}

bool BLHeli::readFirmInfo() {
  memset(&firmInfo, 0, sizeof(firmInfo));

  if (address.versionTag != 0) {
    // XXX: BLHeli_S only.
    uint8_t buf[3];
    if (readFlash(address.versionTag, buf, sizeof(buf)) == false) {
      return false;
    }
    firmInfo.present = true;
    firmInfo.mainRevision = buf[0];
    firmInfo.subRevision = buf[1];
    firmInfo.eepromLayout = buf[2];
  }

  if (address.layoutTag != 0) {
    if (readFlash(address.layoutTag, (uint8_t *)firmInfo.layoutTag,
                  address.layoutTagLen) == false) {
      return false;
    }
  }

  if (address.mcuTag != 0) {
    if (readFlash(address.mcuTag, (uint8_t *)firmInfo.mcuTag,
                  address.mcuTagLen) == false) {
      return false;
    }
  }
  if (address.nameTag != 0) {
    if (readFlash(address.nameTag, (uint8_t *)firmInfo.nameTag,
                  address.nameTagLen) == false) {
      return false;
    }
  }

  return true;
}

BLHeli::escStatus_t BLHeli::get_statusinfo() {
  memset(&status, 0, sizeof(status));
  if (address.status != 0) {
    readSRAM(address.status, (uint8_t *)&status, sizeof(status));
  }

  return status;
}
