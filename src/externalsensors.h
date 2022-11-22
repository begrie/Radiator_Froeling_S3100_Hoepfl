#ifndef __DH_EXTERNALSENSORS_H__
#define __DH_EXTERNALSENSORS_H__

#include "config.h"
#include "network.h"

#include <Ticker.h>
#include <DHT.h>
#include "ESP32_Servo.h"

namespace radiator
{
  class ExternalSensors
  {
  public:
    static bool initExternalSensors();
    // static bool initExternalSensors(bool _dhtGPIO = GPIO_FOR_DHT11,
    //                                 bool _ventilatorRelaisGPIO = GPIO_FOR_VENTILATOR_RELAIS,
    //                                 bool _servoForAirInputFlapGPIO = GPIO_FOR_SERVO_FOR_AIR_INPUT_FLAP,
    //                                 bool _leakWaterSensorGPIO = GPIO_FOR_LEAKWATER_SENSOR,
    //                                 bool _acCurrentSensorGPIO = GPIO_FOR_AC_CURRENT_SENSOR);

    static std::string getSensorValues();
    static std::string getSensorValuesAsJSON();
    static std::string getSensorValueHeaderForCSV();
    static std::string getSensorValueDataForCSV();

    static void setRadiatorIsBurning() { radiatorIsBurning = true; };
    static void setRadiatorFireIsOff() { radiatorIsBurning = false; };

  protected:
    static void xTaskExternalSensors(void *parameter);
    // we need a mutex semaphore to handle concurrent access from different tasks - otherwise "funny" crashes from bufferQueue-handling
    static SemaphoreHandle_t semaphoreExternalSensors;
    // static std::stringstream bufferStringStream; // for all output functions instead to local vars
    static std::string messageBuf;

    static void autoControlVentilatorAndFlap();
    static bool radiatorIsBurning;

    static bool initTempHumidityDHTSensor();
    static int16_t readTemp();
    static int16_t readHumidity();
    // static int16_t getTemp() { return lastRoomTemperature; };
    // static int16_t getHumidity() { return lastRoomHumidity; };
    static int dhtGPIO;
    static DHT tempHumidityDHTSensor;
    static int16_t lastRoomTemperature;
    static int16_t lastRoomHumidity;

    static bool initVentilator();
    static void setVentilatorOn();
    static void setVentilatorOff();
    static int ventilatorRelaisGPIO;
    static bool ventilatorRelaisState;

    static bool initAirInputFlap();
    static void openAirInputFlap();
    static void closeAirInputFlap();
    static int servoForAirInputFlapGPIO;
    static ESP32_Servo servoForAirInputFlap;
    static bool airInputFlapIsOpen;

    static bool initLeakWaterSensor();
    static bool getLeakWaterSensorState() { return leakWaterDetected; };
    static int leakWaterSensorGPIO;
    static volatile bool leakWaterDetected;

    static bool initAcCurrentSensor();
    static void IRAM_ATTR tickerCallbackForReadAndAverageAcCurrentSensor(); // used by ticker
    static double getAcCurrentAmpereRMS();
    static int acCurrentSensorGPIO;
    // static uint16_t scaleForCurrentSensor;  // e.g. 30 Ampere/Volt
    // static uint16_t offsetForCurrentSensor; // due to voltage divider to eliminate negative voltage
    static Ticker tickerForReadAcCurrentSensor;
    static volatile int16_t acCurrentAnalogReadAmplitudeMillivolt;
    // static double acCurrentAmpere;
  };
} // namespace radiator
#endif //#ifndef __DH_EXTERNALSENSORS_H__