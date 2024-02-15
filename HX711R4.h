#include <Arduino.h>

//---------------------------------------------------//
// ロードセル　Ｓ字型　ＳＣ３０１Ａ　１００ｋＧ [P-12036]
//---------------------------------------------------//
//#define OUT_VOL   0.002f      //定格出力 [V]
//#define LOAD      100000.0f   //定格容量 [g]

//---------------------------------------------------//
// ロードセル　シングルポイント（ ビーム型）　ＳＣ６０１　１２０ｋＧ [P-12035]
//---------------------------------------------------//
//#define OUT_VOL   0.001f      //定格出力 [V]
//#define LOAD      120000.0f   //定格容量 [g]

//---------------------------------------------------//
// ロードセル　シングルポイント（ ビーム型）　ＳＣ１３３　２０ｋＧ [P-12034]
//---------------------------------------------------//
//#define OUT_VOL   0.002f      //定格出力 [V]
//#define LOAD      20000.0f    //定格容量 [g]

//---------------------------------------------------//
// ロードセル　シングルポイント（ビーム型）　ＳＣ６１６Ｃ　1ｋｇ[P-17084]
//---------------------------------------------------//
#define OUT_VOL   0.001f   //定格出力 [V] 1[mV] +- 0.15mV/V
#define LOAD      1000.0f  //定格容量 [g]

//---------------------------------------------------//
// ロードセル　シングルポイント（ビーム型）　ＳＣ６１６Ｃ　５００ｇ[P-12532]
//---------------------------------------------------//
//#define OUT_VOL   0.0007f   //定格出力 [V]
//#define LOAD      500.0f    //定格容量 [g]

class AE_HX711 {
  private:
    const static size_t bufferSize = 10;
    pin_size_t pin_data;
    pin_size_t pin_clock;
    float offset;
    bool dataReady;

    long buffer[bufferSize];
    long *bufferp = buffer;
    long MaxValue;
    long minValue;

    static void intr();
    void enableIntr();
    void disableIntr();
    long averaging();

  public:
    AE_HX711();
    ~AE_HX711();

    void init(pin_size_t data, pin_size_t clock);
    void reset();
    long acquire();
    bool isDataReady();

    void tare();
    float getGram();
    float getOffset();
};