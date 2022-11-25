
#include "src/kramotory/kramotory.h"

// declaration:
#define PIN_M1A 18
#define PIN_M1B 5
#define PIN_M2A 21
#define PIN_M2B 19
#define PIN_ENC1A 17
#define PIN_ENC1B 16
#define PIN_ENC2A 22
#define PIN_ENC2B 23

// setting & inicializtion:
KraMotory motor1 = KraMotory(0, PIN_M1A, PIN_M1B, PIN_ENC1A, PIN_ENC1B);
KraMotory motor2 = KraMotory(1, PIN_M2A, PIN_M2B, PIN_ENC2A, PIN_ENC2B);

void setup()
{

    Serial.begin(115200);
     delay(1000);
    Serial.println("Test motory was start.");

    motor1.init();
    motor2.init();
}
void loop()
{

}
void vypis()
{
    static unsigned long oldTime = millis();
    if (millis() - oldTime < 500)
        return;

    Serial.println("loop");
}
