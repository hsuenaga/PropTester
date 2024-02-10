#include "AE_HX711.h"

static AE_HX711 *instance = nullptr;

AE_HX711::AE_HX711()
{

}

AE_HX711::~AE_HX711()
{
  
}

void AE_HX711::intr()
{
  instance->dataReady = true;
}

void AE_HX711::init(pin_size_t data, pin_size_t clock)
{
  dataReady = false;
  pin_data = data;
  pin_clock = clock;

  pinMode(pin_data, INPUT);
  pinMode(pin_clock, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(pin_data), intr, FALLING);
  instance = this;
}

void AE_HX711::reset(void)
{
  digitalWrite(pin_clock,1);
  delayMicroseconds(100);
  digitalWrite(pin_clock,0);
  delayMicroseconds(100); 
}

long AE_HX711::read(void)
{
  long data=0;
  uint8_t data8;

  detachInterrupt(digitalPinToInterrupt(pin_data));
  while(digitalRead(pin_data)!=0);

  for (int i = 0; i < 3; i++) {
    data8 = shiftIn(pin_data, pin_clock, MSBFIRST);
    data = data << 8 | data8;
  }
  // CH A: GAIN = 128
  digitalWrite(pin_clock, 1);
  delayMicroseconds(1);
  digitalWrite(pin_clock, 0);

  if (data & 0x800000) {
    // Sign expantion
    data |= (~0x7fffff);
  }

  dataReady = false;
  attachInterrupt(digitalPinToInterrupt(pin_data), intr, FALLING);
  
  return data; 
}

long AE_HX711::averaging(char num)
{
  long sum = 0;
  for (int i = 0; i < num; i++) sum += read();
  return sum / num;
}

float AE_HX711::getGram(char num)
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

  data = averaging(num); 
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
AE_HX711::tare(void)
{
  offset = getGram(30); 
}

float
AE_HX711::getOffset(void)
{
  return offset;
}

bool
AE_HX711::isDataReady(void)
{
  return dataReady;
}