#include <Arduino.h>
#include <stdarg.h>

#include "HX711R4.h"
#include "DShotR4.h"
#include "MSP.h"
#include "SCS0009.h"

//---------------------------------------------------//
// ピンの設定
//---------------------------------------------------//
#define pin_data  8
#define pin_slk   9
#define pwm_input 2

static HX711 HX711;
static DShotR4 DShot;
static MSP Msp(Serial, DShot);
static SCS0009 SCS(DShot.serialCore);

#undef HX711_TEST
#undef DSHOT_TEST

void setup() {
  Serial.begin(115200);
  while (!Serial);

#ifdef HX711_TEST
  HX711.init(pin_data, pin_slk);
  HX711.reset();
  HX711.tare();
#endif

  DShot.begin(DShotR4::DSHOT300);
  DShot.reset();

  pinMode(pwm_input, INPUT);
}

void
AE_HX711_Print()
{
  char S1[80], TempStr[16];
  double value = HX711.getGram();
  long snapValue;

  snapValue = HX711.acquire();
  dtostrf(value, 5, 3, TempStr);

  snprintf(S1, sizeof(S1), "%s [g] (0x%08x)", TempStr, snapValue);
  Serial.println(S1);
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

  consoleReceive();
}
