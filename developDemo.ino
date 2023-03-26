
#include "src/kramotory/kramotory.h"

// Cifri
// tento text vlozil Cifri

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
KraMotory motorL = KraMotory(0, 1, PIN_M1A, PIN_M1B, PIN_ENC1A, PIN_ENC1B);
KraMotory motorR = KraMotory(2, 3, PIN_M2A, PIN_M2B, PIN_ENC2A, PIN_ENC2B);
#endif

#ifdef KRABOARD
#define PIN_LED1 PA_8
#define PIN_LED2 PA_15
#define PIN_LED3 PD_2

#define PIN_BTN1 PC_4
#define PIN_BTN2 PF4

#define PIN_MOTORLA PC_7
#define PIN_MOTORLB PC_6
#define PIN_MOTORRA PB_8 // 24U 24
#define PIN_MOTORRB PB_9 // 25U 25

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
KraMotory motorL = KraMotory(0, 1, PIN_MOTORLB, PIN_MOTORLA, PIN_ENC_LA, PIN_ENC_LB);
KraMotory motorR = KraMotory(2, 3, PIN_MOTORRB, PIN_MOTORRA, PIN_ENC_RB, PIN_ENC_RA);

#define PIN_AN_BAT PC_5
#endif

// setting & inicializtion:

void testPower()
{
//    Serial.println("Test motorL power is starting. " + String(HAL_GetTick()));
    motorL.setPower(80); // power -255 .. 255
    void vypis();
    delay(5000);
    motorL.setPower(200); // power -255 .. 255
    void vypis();
    delay(5000);
    motorL.setPower(0);
    Serial.println("Test motorR power is starting. " + String(millis()));
    motorR.setPower(80); // power -255 .. 255
    delay(5000);
    motorR.setPower(200); // power -255 .. 255
    delay(5000);
    motorR.setPower(50);
//    Serial.println("Test motors was stoped. " + String(getCurrentMillis()));
    Serial.println("distanceL:" + String(motorL.getDistance()) + " distanceR:" + String(motorR.getDistance()));
    delay(5000);
    motorR.setPower(0);
    void vypis();
}
void testSpeed()
{
    Serial.println("Test motorL speed is starting.");
    motorL.setSpeed(500); // speed v mm/s
    delay(5000);
    motorL.setSpeed(0);
    Serial.println("Test motorR speed is starting.");
    //    motorR.setSpeed(500); // speed v mm/s
    delay(5000);
    motorR.setSpeed(0);
    Serial.println("Test motors was stoped.");
    delay(5000);
}

void vypis()
{
    static unsigned long oldTime = millis();
    if (millis() - oldTime < 500)
    {
        return;
    }
    oldTime = millis();

//    digitalWrite(PIN_LED1, !digitalRead(PIN_LED1));

//    Serial.println("- distanceL:" + String(motorL.getDistance()) + " distanceR:" + String(motorR.getDistance()));
#ifdef USE_STM32
    //    SerialUSB.println("loop");
    SerialUSB.println("distanceL:" + String(motorL.getDistance()) + " distanceR:" + String(motorR.getDistance()));
#endif
#ifdef USE_ESP32
    Serial.println("loop");
#endif
}

void setup()
{
//    pinMode(PIN_LED1,OUTPUT);
//    pinMode(PIN_LED2,OUTPUT);
//    pinMode(PIN_LED3,OUTPUT);
#ifdef USE_STM32
    SerialUSB.begin(115200);
    delay(1000);
    SerialUSB.println("Test motory was start. " + String(millis()));
#endif
#ifdef USE_ESP32
    Serial.begin(115200);
    delay(1000);
    Serial.println("Test motory was start.");
#endif
    //    Serial.begin(115200);
    delay(5000);
    Serial.println("START");
    motorL.init();
    motorR.init();
    delay(2000);
    //    testPower();
    //    testSpeed();
}

void loop()
{
#ifndef USE_FREE_RTOS
//    static unsigned long oldTime = HAL_GetTick();
//    if (HAL_GetTick() - oldTime < 1)
//    {
//        oldTime = HAL_GetTick();
        KraMotory::loop();
//    }

#endif                   // USE_FREE_RTOS
    motorL.setSpeed(100); // speed v mm/s
                         //    motorR.setPower(0); // speed v mm/s
    motorR.setSpeed(50);
    vypis();
}
