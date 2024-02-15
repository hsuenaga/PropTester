#include "HX711R4.h"

static HX711 *instance = nullptr;

HX711::HX711()
{

}

HX711::~HX711()
{
  disableIntr(); 
}

void HX711::intr()
{
  instance->dataReady = true;
}

void HX711::enableIntr()
{
  attachInterrupt(digitalPinToInterrupt(pin_data), intr, FALLING);
}

void HX711::disableIntr()
{
  detachInterrupt(digitalPinToInterrupt(pin_data));
}

void HX711::init(pin_size_t data, pin_size_t clock)
{
  dataReady = false;
  pin_data = data;
  pin_clock = clock;
  memset(buffer, 0, sizeof(buffer));
  MaxValue = 0x007FFFFF;
  minValue = ~MaxValue;

  pinMode(pin_data, INPUT);
  pinMode(pin_clock, OUTPUT);

  instance = this;
  enableIntr();
}

void HX711::reset(void)
{
  digitalWrite(pin_clock,1);
  delayMicroseconds(100);
  digitalWrite(pin_clock,0);
  delayMicroseconds(100); 
}

long HX711::acquire(void)
{
  long data=0;
  uint8_t data8;

  if (!dataReady) {
    return *bufferp;
  }

  disableIntr();
  dataReady = false;

  noInterrupts();  
  for (int i = 0; i < 3; i++) {
    data8 = shiftIn(pin_data, pin_clock, MSBFIRST);
    data = data << 8 | data8;
  }
  // CH A: GAIN = 128
  digitalWrite(pin_clock, 1);
  digitalWrite(pin_clock, 0);
  interrupts();

  enableIntr();

  if (data & 0x800000) {
    // Sign expantion
    data |= (~0x7fffff);
  }
  if (++bufferp >= &buffer[bufferSize]) {
    bufferp = buffer;
  }
  *bufferp = data;

  return data; 
}

long HX711::averaging()
{
  long sum = 0;
  long Max = minValue, min = MaxValue;


  for (int i = 0; i < bufferSize; i++) {
    // assume raw data is 24bit and sum will not be overflowed.
    long val = buffer[i];

    sum += val;
    if (val < min) {
      min = val;
    }
    if (val > Max) {
      Max = val;
    }
  }
  sum -= (min + Max);
  return sum / (bufferSize - 2);
}

float HX711::getGram()
{
  #define HX711_R1  20000.0f
  #define HX711_R2  8200.0f
  #define HX711_VBG 1.25f
  #define HX711_AVDD      4.2987f//(HX711_VBG*((HX711_R1+HX711_R2)/HX711_R2))
  #define HX711_ADC1bit   HX711_AVDD/16777216 //16777216=(2^24)
  #define HX711_PGA 128.0f
  #define HX711_SCALE     (OUT_VOL * HX711_AVDD / LOAD * HX711_PGA)
  
  long data;
  float fdata, ndata, rawV, nV, gram;

  data = averaging(); 
  //Serial.println( HX711_AVDD);   
  //Serial.println( HX711_ADC1bit);   
  //Serial.println( HX711_SCALE);   
  //Serial.println( data);
  fdata = (float)data;
  ndata = fdata / (float)0x7fffff;
  rawV = (HX711_AVDD / HX711_PGA) * ndata;
  nV = rawV / (HX711_AVDD * OUT_VOL);
  gram = LOAD * nV;
#if 0
  Serial.print(" data: ");
  Serial.println(data, HEX);
  Serial.print("fdata: ");
  Serial.println(fdata, 8);
  Serial.print("ndata: ");
  Serial.println(ndata, 8);
  Serial.print("   nV: ");
  Serial.println(nV, 8);
  Serial.print(" rawV: ");
  Serial.println(rawV, 8);
  Serial.print(" gram: ");
  Serial.println(gram);
#endif

  return (gram - offset);
}

void
HX711::tare(void)
{
  int count = 30;
  for (int i = 0; i < 30; i++) {
    while (!dataReady)
      yield();
    acquire();
  }
  offset = getGram(); 
}

float
HX711::getOffset(void)
{
  return offset;
}

bool
HX711::isDataReady(void)
{
  return dataReady;
}