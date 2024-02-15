#include <Arduino.h>

#include "AE_HX711.h"
#include "DShotR4.h"

//---------------------------------------------------//
// ピンの設定
//---------------------------------------------------//
#define pin_dout  8
#define pin_slk   9
#define pin_dshot 0
#define pin_dshotInv 1

static AE_HX711 HX711;

uint32_t gpt_intr_count = 0;
uint32_t gpt_duty = 0;
uint32_t gpt_period = 0;

static DShotR4 DShot;
uint16_t dshot_throttle = 0;

#define DSHOT_TEST

void setup() {
  Serial.begin(115200);

  HX711.init(pin_dout, pin_slk);
  HX711.reset();
  HX711.tare();

  DShot.init(DShotR4::DSHOT300);
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

void loop() 
{ 
  static uint32_t count = 0;
  static unsigned long last_time = 0;

  unsigned long time = millis();
  unsigned long elapsed = time - last_time;

  if (elapsed > 1000) {
#ifdef DSHOT_TEST
    DShot.set_testPattern();
#else
    if (dshot_throttle != 0) {
      DShot.set_throttle(CHANNEL_A, dshot_throttle);
    }
    else {
      DShot.set_command(CHANNEL_A, DShotR4::DSHOT_CMD_DISARM);
    }
#endif
    DShot.transmit();
    last_time = time;
    Serial.println("-----");
    Serial.println(DShot.tx_success);
    Serial.println(DShot.tx_error);
  }

  if (HX711.isDataReady()) 
  {
    HX711.acquire();
//    AE_HX711_Print();
#ifdef ENABLE_DSHOT
    if (count < 10) {
      dshot_throttle = 0;
      count++;
    }
    else if (count < 48) {
      count = 48;
    }
    else if (count < 0x800) {
      dshot_throttle = count;
      count++;
    }
    else {
      count = 0;
    }
#endif
  }
}