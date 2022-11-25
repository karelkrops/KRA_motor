#include "kramotory.h"

//volatile long counter=1;

SemaphoreHandle_t mutexKraMotory;
Motor KraMotory::motory[KRA_MOTORY_MAX_MOTOR] = {};
bool KraMotory::isActiveLoop = false;
TaskHandle_t *KraMotory::rtosTaskHandler = 0;

KraMotory::KraMotory(int idMotor, int GPIO_PWM1, int GPIO_PWM2, int GPIO_enc1 = 0, int GPIO_enc2 = 0)
{
  if (idMotor >= 0 && idMotor < KRA_MOTORY_MAX_MOTOR && !KraMotory::motory[idMotor].isReady)
  {
    this->idMotor = idMotor;
    KraMotory::motory[this->idMotor].GPIO_PWM1 = GPIO_PWM1;
    KraMotory::motory[this->idMotor].GPIO_PWM2 = GPIO_PWM2;
    KraMotory::motory[this->idMotor].GPIO_enc1 = GPIO_enc1;
    KraMotory::motory[this->idMotor].GPIO_enc2 = GPIO_enc2;
    KraMotory::motory[this->idMotor].counter = 0;
    KraMotory::motory[this->idMotor].speed = 0;
    KraMotory::motory[this->idMotor].lastMicros = micros();
  }
  else
  {
    // error
    Serial.println("ERROR: nelze yavest tridu KraMotory. Ma chybne parametry, nebo jiy byla inicializovana.");
  }
}

void KraMotory::rtosTask(void *pvParameters)
{
  (void)pvParameters;
  for (;;)
  {
    loop();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
void KraMotory::init()
{
  if (this->idMotor >= 0 && this->idMotor < KRA_MOTORY_MAX_MOTOR && !KraMotory::motory[idMotor].isReady)
  {
    KraMotory::motory[this->idMotor].counter = 0;
    KraMotory::motory[this->idMotor].speed = 0;
    KraMotory::motory[this->idMotor].lastMicros = micros();
    pinMode(KraMotory::motory[this->idMotor].GPIO_PWM1, OUTPUT);
    pinMode(KraMotory::motory[this->idMotor].GPIO_PWM2, OUTPUT);
    KraMotory::motory[this->idMotor].pulseDistanc = (KraMotory::motory[this->idMotor].wheelDiameter * PI) / KraMotory::motory[this->idMotor].gearRatio; // ujeta delka na pulz

    //       int a=pwm0_out0a;
    if (KraMotory::motory[this->idMotor].GPIO_enc1 > 0)
    {
      pinMode(KraMotory::motory[this->idMotor].GPIO_enc1, INPUT_PULLUP);
      pinMode(KraMotory::motory[this->idMotor].GPIO_enc2, INPUT_PULLUP);
      // nastaveni preruseni
      switch (idMotor)
      {
      case 0:
        attachInterrupt(KraMotory::motory[this->idMotor].GPIO_enc1, isr0, CHANGE);
        attachInterrupt(KraMotory::motory[this->idMotor].GPIO_enc2, isrS0, CHANGE);
        //                attachInterrupt(KraMotory::motory[this->idMotor].GPIO_PWM1, isrPWM0,CHANGE);
        break;
      case 1:
        attachInterrupt(KraMotory::motory[this->idMotor].GPIO_enc1, isr1, CHANGE);
        attachInterrupt(KraMotory::motory[this->idMotor].GPIO_enc2, isrS1, CHANGE);
        break;
      case 2:
        attachInterrupt(KraMotory::motory[this->idMotor].GPIO_enc1, isr2, CHANGE);
        attachInterrupt(KraMotory::motory[this->idMotor].GPIO_enc2, isrS2, CHANGE);
        break;
      case 3:
        attachInterrupt(KraMotory::motory[this->idMotor].GPIO_enc1, isr3, CHANGE);
        attachInterrupt(KraMotory::motory[this->idMotor].GPIO_enc2, isrS3, CHANGE);
        break;
      case 4:
        attachInterrupt(KraMotory::motory[this->idMotor].GPIO_enc1, isr4, CHANGE);
        attachInterrupt(KraMotory::motory[this->idMotor].GPIO_enc2, isrS4, CHANGE);
        break;
      case 5:
        attachInterrupt(KraMotory::motory[this->idMotor].GPIO_enc1, isr5, CHANGE);
        attachInterrupt(KraMotory::motory[this->idMotor].GPIO_enc2, isrS5, CHANGE);
        break;

      default:
        break;
      }
    }
    KraMotory::motory[this->idMotor].isReady = true;
  }
  else
  {
    // error
    Serial.println("ERROR: nelze inicialiyovat tridu KraMotory. Ma chybne parametry, nebo jiz byla inicializovana.");
  }

  if (rtosTaskHandler == 0)
  {
    mutexKraMotory = xSemaphoreCreateMutex();
    if (mutexKraMotory)
    {
      xTaskCreatePinnedToCore(
          KraMotory::rtosTask, "TaskKraMotory" // A name just for humans
          ,
          1024 // This stack size can be checked & adjusted by reading the Stack Highwater
          ,
          0, 2 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
          ,
          rtosTaskHandler, 0);
    }
  }
}
void KraMotory::thisLoop(int idMotor)
{
  //    rychlost je pod regulovatelnym minimem. => zastavit motory
  if (abs(KraMotory::motory[idMotor].requiredSpeed) < KRA_MOTORY_MIN_SPEED && KraMotory::motory[idMotor].speedRegulation)
  {
    setPowerInter(idMotor, 0);
  }

  // ---------------------------START vypocet rychlosti --------------------------------

  // omezeni rychlosti cyklu na spusteni minmalne po jednom pulsu s encoderu
  unsigned long deltaMicrosSpeedControl = micros() - KraMotory::motory[idMotor].lastMicros;
  //    if (KraMotory::motory[idMotor].lastCounter == KraMotory::motory[idMotor].counter &&
  //        deltaMicrosSpeedControl < abs(calcRequiredDeltaMicros(KraMotory::motory[idMotor].requiredSpeed)*3)  )
  //        return;

  //   unsigned long timeBorder = 10000;                                  // hranice casu po jejimy prekroceni motor pravdepodobne stoji
  long deltaMicros = KraMotory::motory[idMotor].deltaMicros;

  if (rtosTaskHandler)
    xSemaphoreTake(mutexKraMotory, portMAX_DELAY);

  if (KraMotory::motory[idMotor].lastCounter == KraMotory::motory[idMotor].counter) // dloho nedoslo k preruseni a aktualizaci deltaMicros
  {
    if (deltaMicrosSpeedControl < abs(calcRequiredDeltaMicros(idMotor, KraMotory::motory[idMotor].requiredSpeed) * 3)) // jeste je to kratka chvyle
    {
      return;
    }
    //   delay(1000);
    deltaMicros = KraMotory::motory[idMotor].dir * (micros() - KraMotory::motory[idMotor].lastMicros);
    KraMotory::motory[idMotor].speed = /* KraMotory::motory[idMotor].dir * */ KraMotory::calcSpeed(idMotor, deltaMicros);
    if (abs(KraMotory::motory[idMotor].speed) < 10)
    {
      KraMotory::motory[idMotor].speed = 0;
    }
  }
  else
  {
    // kdy naposledy byl zaznamenan pohyb
    KraMotory::motory[idMotor].dir = (KraMotory::motory[idMotor].lastCounter <= KraMotory::motory[idMotor].counter) ? 1 : -1;
    KraMotory::motory[idMotor].speed = /* KraMotory::motory[idMotor].dir * */ KraMotory::calcSpeed(idMotor, deltaMicros); // vypocet rychlosti
  }

  KraMotory::motory[idMotor].lastCounter = KraMotory::motory[idMotor].counter;
  if (rtosTaskHandler)
    xSemaphoreGive(mutexKraMotory);

  // ---------------------------END vypocet rychlosti --------------------------------
  if (!KraMotory::motory[idMotor].speedRegulation)
    return; // konec pokud neni rizeno na rychlost

  //  if ( (KraMotory::motory[idMotor].requiredDeltaMicros>=0 && (deltaMicros) > (KraMotory::motory[idMotor].requiredDeltaMicros)) ||
  //  (KraMotory::motory[idMotor].requiredDeltaMicros<0 && (deltaMicros) < (KraMotory::motory[idMotor].requiredDeltaMicros)) )
  if ((KraMotory::motory[idMotor].requiredSpeed >= 0 && (KraMotory::motory[idMotor].speed) < (KraMotory::motory[idMotor].requiredSpeed)) ||
      (KraMotory::motory[idMotor].requiredSpeed < 0 && (KraMotory::motory[idMotor].speed) > (KraMotory::motory[idMotor].requiredSpeed)))
  { // zrychlovani
    double sp = 255;
    if (abs(KraMotory::motory[idMotor].requiredDeltaMicros) > 6000)
      sp = 140;

    KraMotory::setPowerInter(idMotor, sp * ((KraMotory::motory[idMotor].requiredSpeed >= 0) ? 1 : -1));
  }
  else
  { // brzdeni
    double sp = 40;
    if (KraMotory::motory[idMotor].speed != 0 && (KraMotory::motory[idMotor].requiredSpeed) / KraMotory::motory[idMotor].speed < 0.6) // 50% roydil rychlosti
      sp = -20;
    if (KraMotory::motory[idMotor].speed != 0 && (KraMotory::motory[idMotor].requiredSpeed) / KraMotory::motory[idMotor].speed < 0.1) // 50% roydil rychlosti
      sp = -130;
    KraMotory::setPowerInter(idMotor, sp * ((KraMotory::motory[idMotor].requiredSpeed >= 0) ? 1 : -1));
  }

  return;
}
void KraMotory::acceleration(int idMotor, double deltaMicrosAccelerationRegulation)
{
  // akcelerace
  double x;

  if (rtosTaskHandler)
    xSemaphoreTake(mutexKraMotory, portMAX_DELAY);

  if (KraMotory::motory[idMotor].requiredUserSpeed > 0)
  { // dopredu
    if (KraMotory::motory[idMotor].requiredUserSpeed > KraMotory::motory[idMotor].requiredSpeed)
    {
      x = KraMotory::motory[idMotor].requiredSpeed + (KraMotory::motory[idMotor].accelerationFront * 0.000001 * deltaMicrosAccelerationRegulation);
      if (x > KraMotory::motory[idMotor].requiredUserSpeed)
        x = KraMotory::motory[idMotor].requiredUserSpeed;
    }
    else
    { // zpomaleni
      x = KraMotory::motory[idMotor].requiredSpeed - (KraMotory::motory[idMotor].breakFront * 0.000001 * deltaMicrosAccelerationRegulation);
      if (x < KraMotory::motory[idMotor].requiredUserSpeed)
        x = KraMotory::motory[idMotor].requiredUserSpeed;
    }
  }
  else
  { // dozadu
    if (KraMotory::motory[idMotor].requiredUserSpeed < KraMotory::motory[idMotor].requiredSpeed)
    { // zrychleni
      x = KraMotory::motory[idMotor].requiredSpeed - (KraMotory::motory[idMotor].accelerationBack * 0.000001 * deltaMicrosAccelerationRegulation);
      if (x < KraMotory::motory[idMotor].requiredUserSpeed)
        x = KraMotory::motory[idMotor].requiredUserSpeed;
    }
    else
    { // zpomaleni
      x = KraMotory::motory[idMotor].requiredSpeed + (KraMotory::motory[idMotor].breakBack * 0.000001 * deltaMicrosAccelerationRegulation);
      if (x > KraMotory::motory[idMotor].requiredUserSpeed)
        x = KraMotory::motory[idMotor].requiredUserSpeed;
    }
  }
  if (rtosTaskHandler)
    xSemaphoreGive(mutexKraMotory);

  setSpeedInter(idMotor, x);
}
void KraMotory::loop()
{
  static unsigned long oldMicros = micros();
  const double deltaMicrosAccelerationRegulation = 1000;
  KraMotory::isActiveLoop = true;
  for (size_t i = 0; i < KRA_MOTORY_MAX_MOTOR; i++)
  {
    if (KraMotory::motory[i].isReady)
    {
      KraMotory::thisLoop(i);
    }
  }
  const double delta = micros() - oldMicros;
  if (delta < deltaMicrosAccelerationRegulation)
    return;

  for (size_t i = 0; i < KRA_MOTORY_MAX_MOTOR; i++)
  {
    if (KraMotory::motory[i].isReady)
    {
      KraMotory::acceleration(i, delta);
    }
  }
  oldMicros = micros();
}

double KraMotory::calcSpeed(int idMotor, long deltaMicros)
{
  //  KraMotory::motory[idMotor].pulseDistanc = (KraMotory::motory[idMotor].wheelDiameter*PI)/KraMotory::motory[idMotor].gearRatio; // ujeta delka na pulz
  return (1000000 * KraMotory::motory[idMotor].pulseDistanc) / double(deltaMicros);
};
long KraMotory::calcRequiredDeltaMicros(int idMotor, double pSpeed)
{
  if (abs(pSpeed) > KRA_MOTORY_MIN_SPEED)
    return long((1000000 * KraMotory::motory[idMotor].pulseDistanc) / pSpeed);
  else
    return long((1000000 * KraMotory::motory[idMotor].pulseDistanc) / KRA_MOTORY_MIN_SPEED);
};

void KraMotory::setPowerInter(int idMotor, double requiredPower)
{
  KraMotory::motory[idMotor].aktPower = requiredPower;
  int power = abs(int(requiredPower));
  if (power > 255)
    power = 255;
  if (requiredPower >= 0)
  {
    //        analogWrite(KraMotory::motory[idMotor].GPIO_PWM1, 0U,255U);
    //        analogWrite(KraMotory::motory[idMotor].GPIO_PWM2, power,255);
    analogWrite(KraMotory::motory[idMotor].GPIO_PWM2, 255U, 255U);
    analogWrite(KraMotory::motory[idMotor].GPIO_PWM1, 255U - power, 255);
  }
  else
  {
    //        analogWrite(KraMotory::motory[idMotor].GPIO_PWM1, power,255U);
    //        analogWrite(KraMotory::motory[idMotor].GPIO_PWM2, 0U,255U);
    analogWrite(KraMotory::motory[idMotor].GPIO_PWM2, 255U - power, 255U);
    analogWrite(KraMotory::motory[idMotor].GPIO_PWM1, 255U, 255U);
  }
}
void KraMotory::setSpeedInter(int idMotor, double requiredSpeed)
{
  KraMotory::motory[idMotor].requiredSpeed = requiredSpeed;
  KraMotory::motory[idMotor].requiredDeltaMicros = calcRequiredDeltaMicros(idMotor, requiredSpeed);
}

KraMotory::~KraMotory()
{
  if (KraMotory::motory[this->idMotor].GPIO_enc1 > 0)
  {
    detachInterrupt(KraMotory::motory[this->idMotor].GPIO_enc1);
  }
}

/**
* @brief vrati momentalni rychlost v mm/s
* 
* @return double 
*/
double KraMotory::getSpeed()
{
  double i;
  if (rtosTaskHandler)
    xSemaphoreTake(mutexKraMotory, portMAX_DELAY);
  i = KraMotory::motory[this->idMotor].speed;
  if (rtosTaskHandler)
    xSemaphoreGive(mutexKraMotory);
  return i;
}
void KraMotory::setSpeed(int idMotor, double requiredSpeed)
{
  if (rtosTaskHandler)
    xSemaphoreTake(mutexKraMotory, portMAX_DELAY);
  KraMotory::motory[idMotor].requiredUserSpeed = requiredSpeed;
  KraMotory::motory[idMotor].speedRegulation = true;
  if (rtosTaskHandler)
    xSemaphoreGive(mutexKraMotory);
}
void KraMotory::setPower(int idMotor, double requiredPower)
{
  if (rtosTaskHandler)
    xSemaphoreTake(mutexKraMotory, portMAX_DELAY);
  KraMotory::motory[idMotor].speedRegulation = false;
  setPowerInter(idMotor, requiredPower);
  if (rtosTaskHandler)
    xSemaphoreGive(mutexKraMotory);
}
/**
     * @brief nastaveni vykonu v rozssahu -255 az 255
     * 
     * @param requiredPower 
     */
void KraMotory::setPower(float requiredPower) { KraMotory::setPower(this->idMotor, requiredPower); }
/**
     * @brief nastaveni speed v mm/s
     * 
     * @param requiredSpeed 
     */
void KraMotory::setSpeed(float requiredSpeed) { KraMotory::setSpeed(this->idMotor, requiredSpeed); }
/**
     * @brief reset ujete vydalenosti
     * 
     */
void KraMotory::resetDistance()
{
  if (rtosTaskHandler)
    xSemaphoreTake(mutexKraMotory, portMAX_DELAY);
  KraMotory::motory[this->idMotor].counterNullDistance = KraMotory::motory[this->idMotor].counter;
  if (rtosTaskHandler)
    xSemaphoreGive(mutexKraMotory);
}
/**
     * @brief vrati ujetou vydalenost v mm
     * 
     * @return double 
     */
double KraMotory::getDistance()
{
  double i;
  if (rtosTaskHandler)
    xSemaphoreTake(mutexKraMotory, portMAX_DELAY);
  i = (KraMotory::motory[this->idMotor].counter - KraMotory::motory[this->idMotor].counterNullDistance) * KraMotory::motory[this->idMotor].pulseDistanc;
  if (rtosTaskHandler)
    xSemaphoreGive(mutexKraMotory);
  return i;
}
/**
     * @brief nastaveni polomeru kola v mm
     * 
     * @param wheelDiameter 
     */
void KraMotory::setWheelDiameter(double wheelDiameter)
{
  if (rtosTaskHandler)
    xSemaphoreTake(mutexKraMotory, portMAX_DELAY);
  KraMotory::motory[this->idMotor].wheelDiameter = wheelDiameter;
    KraMotory::motory[this->idMotor].pulseDistanc = (KraMotory::motory[this->idMotor].wheelDiameter * PI) / KraMotory::motory[this->idMotor].gearRatio; // ujeta delka na pulz
  if (rtosTaskHandler)
    xSemaphoreGive(mutexKraMotory);
}
/**
     * @brief nastaveni poctu inpulsu na otacku, pocitaji se nabehove i sestupne hranz obou encoderu
     * pomer prevodu 30 * pocet magnetickzch pulsu 3 * pocet hran na magnet 4 = 360 pulsu na otacku
     * @param gearRatio 
     */
void KraMotory::setGearRatio(int gearRatio)
{
  if (rtosTaskHandler)
    xSemaphoreTake(mutexKraMotory, portMAX_DELAY);
  KraMotory::motory[this->idMotor].gearRatio = gearRatio;
    KraMotory::motory[this->idMotor].pulseDistanc = (KraMotory::motory[this->idMotor].wheelDiameter * PI) / KraMotory::motory[this->idMotor].gearRatio; // ujeta delka na pulz
  if (rtosTaskHandler)
    xSemaphoreGive(mutexKraMotory);
}
void KraMotory::setAccelerationFront(double i)
{
  if (rtosTaskHandler)
    xSemaphoreTake(mutexKraMotory, portMAX_DELAY);
  KraMotory::motory[this->idMotor].accelerationFront = i;
  if (rtosTaskHandler)
    xSemaphoreGive(mutexKraMotory);
}
void KraMotory::setAccelerationBack(double i)
{
  if (rtosTaskHandler)
    xSemaphoreTake(mutexKraMotory, portMAX_DELAY);
  KraMotory::motory[this->idMotor].accelerationBack = i;
  if (rtosTaskHandler)
    xSemaphoreGive(mutexKraMotory);
}
void KraMotory::setBreakFront(double i)
{
  if (rtosTaskHandler)
    xSemaphoreTake(mutexKraMotory, portMAX_DELAY);
  KraMotory::motory[this->idMotor].breakFront = i;
  if (rtosTaskHandler)
    xSemaphoreGive(mutexKraMotory);
}
void KraMotory::setBreakBack(double i)
{
  if (rtosTaskHandler)
    xSemaphoreTake(mutexKraMotory, portMAX_DELAY);
  KraMotory::motory[this->idMotor].breakBack = i;
  if (rtosTaskHandler)
    xSemaphoreGive(mutexKraMotory);
}

// ----------------------------interupt------------------------------------------------------------
void IRAM_ATTR isr(int idMotor)
{
  //   KraMotory::motory[idMotor].counter = KraMotory::motory[idMotor].counter + 1;
  //return;

  if (digitalRead(KraMotory::motory[idMotor].GPIO_enc1) == digitalRead(KraMotory::motory[idMotor].GPIO_enc2))
  {
    KraMotory::motory[idMotor].counter = KraMotory::motory[idMotor].counter + 1;
    KraMotory::motory[idMotor].deltaMicros = (micros() - KraMotory::motory[idMotor].lastMicros);
  }
  else
  {
    KraMotory::motory[idMotor].counter = KraMotory::motory[idMotor].counter - 1;
    KraMotory::motory[idMotor].deltaMicros = (micros() - KraMotory::motory[idMotor].lastMicros) * -1;
  }

  KraMotory::motory[idMotor].lastMicros = micros();
}
void IRAM_ATTR isrS(int idMotor)
{
  //   KraMotory::motory[idMotor].counter = KraMotory::motory[idMotor].counter + 1;
  //return;

  if (digitalRead(KraMotory::motory[idMotor].GPIO_enc1) != digitalRead(KraMotory::motory[idMotor].GPIO_enc2))
  {
    KraMotory::motory[idMotor].counter = KraMotory::motory[idMotor].counter + 1;
    KraMotory::motory[idMotor].deltaMicros = (micros() - KraMotory::motory[idMotor].lastMicros);
  }
  else
  {
    KraMotory::motory[idMotor].counter = KraMotory::motory[idMotor].counter - 1;
    KraMotory::motory[idMotor].deltaMicros = (micros() - KraMotory::motory[idMotor].lastMicros) * -1;
  }

  KraMotory::motory[idMotor].lastMicros = micros();
}
