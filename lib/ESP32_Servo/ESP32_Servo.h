/******************************************************************************
 * Name:		ESP32_Servo.h
 * @brief   Control of "standard" servos ONLY on ESP using PWM generation with LEDC unit
 *          special feature: speed controlled servo movement
 * Created:	26.10.2022 (full refactor of 2017 version)
 * Author:	BeGrie
 ******************************************************************************/

#ifndef ESP32_Servo_h
#define ESP32_Servo_h

#include <Arduino.h>
#include <Ticker.h>

/***********************
 *      DEFINES
 ***********************/
#define DEBUG_ESP32_SERVO false

#define SERVO_AUTO_DEACTIVATION_DELAY_MS 1500    // delay for automatic servo deactivation -> ONLY USED FOR FULL SPEED POSITIONING
                                                 // should be as long as the max. positioning time from 0 to 180° lasts for the specific servo under load
                                                 // 0 to disable auto deactivation
#define SERVO_TARGET_TIME_MS_FOR_180_DEGREE 3000 // definition for STANDARD-SPEED ONLY IN moveToAngleWithSpeedControl()
                                                 // can also be calculated with  (SERVO_ANGLESPEED_DEGREE_PER_SECOND * 180 / 1000)
#define SERVO_ANGLESPEED_DEGREE_PER_SECOND (180 * 1000 / SERVO_TARGET_TIME_MS_FOR_180_DEGREE)

#define MAX_LEDC_CHANNELS 16                          // depends on ESP32 version
#define AUTOSELECT_LEDC_CHANNEL_FROM_MAX_TO_MIN false // true: start with highest ledc channel
                                                      // -> evtl. to avoid timer conflicts with other application parts

// servo configuration -> fits for standard servos:
#define SERVO_MIN_PULSEWIDTH 544 // 544 µs -> corresponds to 0°
#define SERVOANGLE_AT_MIN_PULSEWIDTH 0
#define SERVO_MAX_PULSEWIDTH 2400 // 2400 µs ->corresponds to 180°
#define SERVOANGLE_AT_MAX_PULSEWIDTH 180
#define SERVO_PWM_FREQUENCY_HZ 50 // for servos a standard refresh cycle (ledc duty cycle) is 20ms -> 50Hz

/***********************
 * CLASS DECLARATION
 ***********************/
class ESP32_Servo
{
public:
  ESP32_Servo(uint8_t _servoPin, uint8_t _ledcChannel = AUTOSELECT_LEDC_CHANNEL, uint8_t _minAngle = SERVOANGLE_AT_MIN_PULSEWIDTH, uint8_t _maxAngle = SERVOANGLE_AT_MAX_PULSEWIDTH, uint32_t _targetTimeMs = SERVO_TARGET_TIME_MS_FOR_180_DEGREE, uint16_t _autoDeactivationDelayMs = SERVO_AUTO_DEACTIVATION_DELAY_MS);

  ~ESP32_Servo();

  void moveToAngle(uint8_t _targetAngle); // only fullspeed
  void moveToAngleWithSpeedControl(uint8_t _targetAngle, uint16_t _delayMsPerDegree = DELAY_FROM_TARGET_TIME);

  void moveToMaxAngle() { moveToAngle(maxAngle); };
  void moveToMinAngle() { moveToAngle(minAngle); };
  void moveToMaxAngleWithSpeedControl(uint16_t _delayMsPerDegree = DELAY_FROM_TARGET_TIME) { moveToAngleWithSpeedControl(maxAngle, _delayMsPerDegree); };
  void moveToMinAngleWithSpeedControl(uint16_t _delayMsPerDegree = DELAY_FROM_TARGET_TIME) { moveToAngleWithSpeedControl(minAngle, _delayMsPerDegree); };

  void sleep() { deactivate(); };

  void setServoPin(uint8_t _servoPin); // only needed if not known at construction or changed

  void setMinMaxAngles(uint8_t _minAngle, uint8_t _maxAngle);

  uint16_t setTargetTime(uint32_t _targetTimeMs);                             // use this ...
  uint16_t setAnglespeedDegreePerSecond(uint16_t _anglespeedDegreePerSecond); //...OR this to set the desired servo speed

  uint8_t getTargetAngle() { return targetAngle; };
  uint8_t getActualAngle() { return actualAngle; };
  bool getIsMoving() { return tickerForSpeedControl.active() || tickerForAutoDeactivation.active(); };

protected:
  bool initLEDCforPWM(uint32_t _pwmFrequency = SERVO_PWM_FREQUENCY_HZ, uint8_t _dutyCycleResolutionBits = LEDC_DUTY_CYCLE_RESOLUTION_BITS);
  uint8_t autoselectLedcChannel(bool _fromMaxToMin = AUTOSELECT_LEDC_CHANNEL_FROM_MAX_TO_MIN);

  uint16_t calcDutyForServoAngle(uint8_t _servoAngle);
  uint8_t checkforMinMaxAngle(uint8_t _angleToCheck);

  void activate();
  void deactivate();

  bool isActivated = false;
  uint16_t autoDeactivationDelayMs; // 0 to disable
  Ticker tickerForAutoDeactivation;
  static void callbackTickerForAutoDeactivation(ESP32_Servo *ptrServo);

  Ticker tickerForSpeedControl;
  static void callbackTickerForSpeedControl(ESP32_Servo *ptrServo);

  uint8_t servoPin;
  uint8_t ledcChannel;              // LEDC PWM channel used (permanently bound to a timer channel in the Arduino library)
                                    // Assignment to servos and freedom from overlapping with other timer functions must be ensured by the application
  static bool usedLedcChannels[16]; // for all instances logging of used ledcChannels for automatic assignments

  uint8_t minAngle; //"End stops" of the servo angle
  uint8_t maxAngle; // must be within 0 to 180 degrees (resp. SERVOANGLE_AT_MIN_PULSEWIDTH to SERVOANGLE_AT_MAX_PULSEWIDTH)

  uint8_t targetAngle = 90;
  uint8_t actualAngle = 90;

  uint32_t targetTimeMs;                       //"Desired" positioning time in ms for a movement from Min_Angle to Max_Angle
                                               // Value 0 -> no delay
  uint16_t delayMsPerDegreeFromTargetTime = 1; // is calculated from targetTimeMs
  uint16_t anglespeedDegreePerSecond = 0;      // is calculated from targetTimeMs

  enum constants // do not change!
  {
    DELAY_FROM_TARGET_TIME = 0,
    AUTOSELECT_LEDC_CHANNEL = 255, // 255 (exact: values larger than MAX_LEDC_CHANNELS)
                                   // signals to autoselect a free ledcChannel for PWM generation from the library
    SERVO_PWM_INTERVAL_MICROSEC =
        (1000 * 1000) / SERVO_PWM_FREQUENCY_HZ, // minimum time to refresh servos in microseconds
                                                // -> corresponds to "duty-cycle" in LEDC-Lib of ESP32 (at 50Hz -> 20000 microsec)
    LEDC_DUTY_CYCLE_RESOLUTION_BITS = 12,       // Resolution of the timer in bits within one period of the servo refresh interval
                                                // 12 bit -> 4096 steps over...
                                                //    - one duty cycle of 20 ms -> minimal increment/resolution = 4.88 microseconds
                                                //    - 2400µs - 544µs = 1856µs / 4.88µs = 380 increments for 180 degree
                                                //    -> ca. 0,5° angle resolution
    LEDC_TIMER_RESOLUTION_STEPS = 4096          // Resolution of the timer in steps (->2^LEDC_DUTY_CYCLE_RESOLUTION_BITS)
  };
};

#endif // von ESP32_Servo_h