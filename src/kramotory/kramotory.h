#ifndef KRA_MOTORY
#define KRA_MOTORY
/**
 * @autor KRA Pisek  Karel Krops
 * pro platformu ESP32
 * konstrukce F1 z roku 2021
 * maximalni pocet motoru je 6
 * poslední aktualizace 1.1.2023
 * verze 1.5.0
 */

/** HELP
 * 
 * ****************************************
 // declaration:
#define PIN_M1A 18
#define PIN_M1B 5
#define PIN_M2A 21
#define PIN_M2B 19
#define PIN_ENC1A 17
#define PIN_ENC1B 16
#define PIN_ENC2A 22
#define PIN_ENC2B 23

#include "kramotory.h"
 
 // setting & inicializtion:
 KraMotory motor1= KraMotory(0,1,PIN_M1A,PIN_M1B,PIN_ENC1A,PIN_ENC1B); // analog chanel 0..15 
 KraMotory motor2= KraMotory(2,3,PIN_M2A,PIN_M2B,PIN_ENC2A,PIN_ENC2B);
  
 // setup
  motor1.setWheelDiameter(45); // prumer kola 45 mm
  motor2.setWheelDiameter(45); // prumer kola 45 mm
  motor1.setGearRatio(30*3*4); // prevodovz pomer 30, sestipolovy magnet (3* sever a 3* jih), polsu s enkoderu na magnet 4
  motor2.setGearRatio(30*3*4); // prevodovz pomer 30, sestipolovy magnet (3* sever a 3* jih), polsu s enkoderu na magnet 4
  motor1.init();
  motor2.init();
  motor1.setAccelerationFront(20000); // dopredne zrychleni v mm/s^2
  motor2.setAccelerationFront(20000); // dopredne zrychleni v mm/s^2
  motor1.setAccelerationBack(20000); // zrychleni pri couvani zrychleni v mm/s^2
  motor2.setAccelerationBack(20000); // zrychleni pri couvani zrychleni v mm/s^2
  motor1.setBreakFront(20000); // dopredne brzdeni v mm/s^2
  motor2.setBreakFront(20000); // dopredne brzdeni v mm/s^2
  motor1.setBreakBack(20000); // brzdeni pri couvani v mm/s^2
  motor2.setBreakBack(20000); // brzdeni pri couvani v mm/s^2
  
 // use:
  motor1.setSpeed(power); // speed v mm/s
  motor2.setSpeed(power); // speed v mm/s 
  motor1.setPower(speed); // power -255 .. 255
  motor2.setPower(speed); // power -255 .. 255
 
  motor1.getSpeed(); // vrati aktualni absolutni rychlost
  motor2.getSpeed(); // vrati aktualni absolutni rychlost
  motor1.resetDistance(); // reset ujete vzdalenosti
  double distance = motor1.getDistance(); // precteni ujete vzdalenosti
  motor2.resetDistance(); // reset ujete vzdalenosti
  double distance = motor2.getDistance(); // precteni ujete vzdalenosti

 * *******************************************
 */

#include <Arduino.h>

//#define USE_FREE_RTOS

#define KRA_MOTORY_MAX_MOTOR 6    // maximalni pocet motoru
#define KRA_MOTORY_MAX_SPEED 3500 // rychlost je merena v [mm / sekund]
#define KRA_MOTORY_MIN_SPEED 10 // rychlost je merena v [mm / sekund] je dulezite pro detekci stojicich motoru
#define KRA_MOTORY_MIN_POWER 50  // minimalni vykon na minimalni pohyb kol
#define KRA_AccelerationFront 20000
#define KRA_AccelerationBack 20000
#define KRA_BreakFront 20000
#define KRA_BreakBack 20000

#ifdef ARDUINO_BLUEPILL_F103C8
# define USE_STM32
#endif
#ifdef ARDUINO_ARCH_STM32
# define USE_STM32
#endif

#ifdef ARDUINO_ARCH_ESP32
# define USE_ESP32
#endif

#ifdef USE_FREE_RTOS
# ifdef USE_STM32
#  include <STM32FreeRTOS.h>
//#  include <FreeRTOS.h>
# endif
#endif

// popis interupt in ESP32
// https://lastminuteengineers.com/handling-esp32-gpio-interrupts-tutorial/

// popis FreeRtos
// https://dspace.cvut.cz/bitstream/handle/10467/80334/F3-DP-2019-Kral-Vaclav-kralvac7.pdf?sequence=-1&isAllowed=y
// https://otik.zcu.cz/bitstream/11025/13949/1/DIPLOMOVA%20PRACE.pdf
// https://circuitdigest.com/microcontroller-projects/arduino-freertos-tutorial-using-semaphore-and-mutex-in-freertos-with-arduino

//#include <analogWrite.h> // https://github.com/ERROPiX/ESP32_AnalogWrite
// alternative https://randomnerdtutorials.com/esp32-pwm-arduino-ide/

//extern volatile long counter;
/**
 * @brief parametry jednotlivych motoru
 * parameters of individual engines
 * 
 */
struct Motor
{
    /** GPIO of engine */
    int GPIO_PWM1, GPIO_PWM2, GPIO_enc1, GPIO_enc2;
    int GPIO_chanelPWM1,GPIO_chanelPWM2;
    /** prumer kola [mm] */
    double wheelDiameter = 45;
    /** pocet pulzu encoderu na otacku. Pocitaji se obe nabehove i sestupne hrany */
    int gearRatio = 30*3*4;
    /** pocitadlo pulzu */
    double pulseDistanc = (wheelDiameter*PI)/(30*3*4);
    volatile long counter = 0;
    double counterNullDistance = 0;
    volatile long lastCounter = 0; // posledni stav pocitadla pulzu
    double speed = 0; // rychlost [mm/s]
    int dir=1; // smer pohzbu motoru
    double accelerationFront = KRA_AccelerationFront;  // dopredne zrychleni v mm/s^2
    double accelerationBack = KRA_AccelerationBack;  // zrychleni pri couvani zrychleni v mm/s^2
    double breakFront = KRA_BreakFront;  // dopredne brzdeni v mm/s^2
    double breakBack = KRA_BreakBack;  // brzdeni pri couvani v mm/s^2
    bool isReady = false;
    bool speedRegulation = false;
    volatile unsigned long lastMicros = 0; // cas posledniho preruseni
    volatile long deltaMicros = 0; // pri couvani ma zapornou hodnotu
//    volatile unsigned long lastMicrosSpeedControl = 0; 
    double requiredUserSpeed =0;
    double requiredSpeed = 0;  // pozadovana rychlost kladna i zaporna v [mm/s]
    long requiredDeltaMicros = 0; // pri couvani ma zapornou hodnotu
    double maxSpeed = KRA_MOTORY_MAX_SPEED;
    double minPower = KRA_MOTORY_MIN_POWER;
    double aktPower = 0;
//    float PID_i = 0;
//    float PID_iArray[10] = {};
//    float PID_iStep = 0.0;
//    float PID_kd = 0.0;
//    float PID_kp = 255 / KRA_MOTORY_MAX_SPEED;
};
/**
 * @brief trida ovladani motoru
 * 
 */
class KraMotory
{
private:
#ifdef USE_FREE_RTOS
    static TaskHandle_t *rtosTaskHandler;
    static void rtosTask(void *pvParameters);
#endif // USE_FREE_RTOS
    static int counterIdMotoru;
    static bool isActiveLoop;
    static double calcSpeed(int idMotor, long deltaMicros);
    static long calcRequiredDeltaMicros(int idMotor, double pSpeed);
    public:
    static void setPowerInter(int idMotor, double requiredPower);
    static void setSpeedInter(int idMotor, double requiredSpeed);
    static void thisLoop(int idMotor);
    static void acceleration(int idMotor, double deltaMicrosAccelerationRegulation);
    static void loop();
    static Motor motory[];
    int idMotor;
public:
    void setAccelerationFront(double i=KRA_AccelerationFront);
    void setAccelerationBack(double i=KRA_AccelerationBack);
    void setBreakFront(double i=KRA_BreakFront);
    void setBreakBack(double i=KRA_BreakBack);
    KraMotory(int GPIO_chanelPWM1, int GPIO_chanelPWM2, int GPIO_PWM1, int GPIO_PWM2, int GPIO_enc1, int GPIO_enc2);
    ~KraMotory();
    void setWheelDiameter(double wheelDiameter);
    void setGearRatio(int gearRatio);
    void init();
    void setSpeed(float requiredSpeed);
    void setPower(float requiredPower);
    Motor getMotor(){return this->motory[this->idMotor];}
    static void setPower(int idMotor, double requiredPower);
    static void setSpeed(int idMotor, double requiredSpeed);
    double getSpeed();
    void resetDistance();
    double getDistance();
};

#ifdef USE_ESP32
void IRAM_ATTR isr(int idMotor);
inline void IRAM_ATTR isr0() { isr(0); }
inline void IRAM_ATTR isr1() { isr(1); }
inline void IRAM_ATTR isr2() { isr(2); }
inline void IRAM_ATTR isr3() { isr(3); }
inline void IRAM_ATTR isr4() { isr(4); }
inline void IRAM_ATTR isr5() { isr(5); }
void IRAM_ATTR isrS(int idMotor);
inline void IRAM_ATTR isrS0() { isrS(0); }
inline void IRAM_ATTR isrS1() { isrS(1); }
inline void IRAM_ATTR isrS2() { isrS(2); }
inline void IRAM_ATTR isrS3() { isrS(3); }
inline void IRAM_ATTR isrS4() { isrS(4); }
inline void IRAM_ATTR isrS5() { isrS(5); }
#endif

#ifdef USE_STM32
void isr(int idMotor);
inline void isr0() { isr(0); }
inline void isr1() { isr(1); }
inline void isr2() { isr(2); }
inline void isr3() { isr(3); }
inline void isr4() { isr(4); }
inline void isr5() { isr(5); }
void isrS(int idMotor);
inline void isrS0() { isrS(0); }
inline void isrS1() { isrS(1); }
inline void isrS2() { isrS(2); }
inline void isrS3() { isrS(3); }
inline void isrS4() { isrS(4); }
inline void isrS5() { isrS(5); }
#endif


#endif //KRA_MOTORY
