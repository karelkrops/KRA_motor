
#include "src/kramotory/kramotory.h"

#ifdef ARDUINO_ARCH_STM32
#define KRABOARD
#endif

#ifndef KRABOARD
// declaration:
#define PIN_M1A 18
#define PIN_M1B 5
#define PIN_M2A 21
#define PIN_M2B 19
#define PIN_ENC1A 17
#define PIN_ENC1B 16
#define PIN_ENC2A 22
#define PIN_ENC2B 23
KraMotory motor1 = KraMotory(0,1, PIN_M1A, PIN_M1B, PIN_ENC1A, PIN_ENC1B);
KraMotory motor2 = KraMotory(2,3, PIN_M2A, PIN_M2B, PIN_ENC2A, PIN_ENC2B);
#endif

#ifdef KRABOARD
#define PIN_LED1 PA_8
#define PIN_LED2 PA_15
#define PIN_LED3 PD_2

#define PIN_BTN1 PC_4
#define PIN_BTN2 PF4

#define PIN_MOTORLA PC_7
#define PIN_MOTORLB PC_6
#define PIN_MOTORRA PB_8
#define PIN_MOTORRB PB_9

#define PIN_ENC_LA PA_4
#define PIN_ENC_LB PA_5
#define PIN_ENC_RA PC_12
#define PIN_ENC_RB PC_13

#define PIN_AN_LINE1 PC_0
#define PIN_LED_LINE1 PC_1
#define PIN_AN_LINE2 PC_2
#define PIN_LED_LINE2 PC_3

#define PIN_MX0 PB_15
#define PIN_MX1 PB_14
#define PIN_MX2 PB_13
#define PIN_MX3 PB_12
KraMotory motor1 = KraMotory(0,1, PIN_MOTORLA, PIN_MOTORLB, PIN_ENC_LA, PIN_ENC_LB);
KraMotory motor2 = KraMotory(2,3, PIN_MOTORRA, PIN_MOTORRB, PIN_ENC_RA, PIN_ENC_RB);

#define PIN_AN_BAT PC_5
#endif

// setting & inicializtion:

void testPower()
{
    Serial.println("Test motor1 power is starting.");
    motor1.setPower(100); // power -255 .. 255
    delay(5000);
    motor1.setPower(0);
    Serial.println("Test motor2 power is starting.");
    motor2.setPower(100); // power -255 .. 255
    delay(5000);
    motor2.setPower(0);
    Serial.println("Test motors was stoped.");
    delay(5000);
}
void testSpeed()
{
    Serial.println("Test motor1 speed is starting.");
    motor1.setSpeed(500); // speed v mm/s
    delay(5000);
    motor1.setSpeed(0);
    Serial.println("Test motor2 speed is starting.");
    motor2.setSpeed(500); // speed v mm/s
    delay(5000);
    motor2.setSpeed(0);
    Serial.println("Test motors was stoped.");
    delay(5000);
}

void setup()
{

    Serial.begin(115200);
    delay(1000);
    Serial.println("Test motory was start.");

    motor1.init();
    motor2.init();
    delay(2000);
    testPower();
    testSpeed();
}
void loop()
{
#ifndef USE_FREE_RTOS
    KraMotory::loop();
#endif // USE_FREE_RTOS
    

}
void vypis()
{
    static unsigned long oldTime = millis();
    if (millis() - oldTime < 500)
        return;

    Serial.println("loop");
}
