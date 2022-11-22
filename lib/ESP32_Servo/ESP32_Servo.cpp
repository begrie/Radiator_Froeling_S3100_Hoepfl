#include "ESP32_Servo.h"

bool ESP32_Servo::usedLedcChannels[16]{false};

/*********************************************************************
 * @brief 	Constructor
 * @param 	GPIO for Servo -> is only used but not checked!
 * @param 	LEDC channel -> is not checked for double occupation and side effects with other timer using functions!
 *          // -> must be validated by lib-using application
 * @param   minimum Angle for Servo to drive to
 * @param   maximum Angle for Servo to drive to
 * @param   desired target time in ms to drive from minAngle to maxAngle (and vice versa) to slow down servo movement for moveToAngleWithSpeedControl()
 *          // makes only sense when time is longer than "natural time" of servo movement
 * @param   delay in ms for automatic deactivation of servo after positioning without speed control -> saves energy and avoid noise from servo
 *          -> a reasonable value itself is only needed for full speed servo movements with moveToAngle() and should be longer than "natural time" of servo movement
 *          -> for moveToAngleWithSpeedControl() is only a value != 0 needed to deactivate after the last movement increment
 * @return 	void
 *********************************************************************/
ESP32_Servo::ESP32_Servo(uint8_t _servoPin, uint8_t _ledcChannel, uint8_t _minAngle, uint8_t _maxAngle, uint32_t _targetTimeMs, uint16_t _autoDeactivationDelayMs)
    : servoPin(_servoPin),
      ledcChannel(_ledcChannel),
      minAngle(_minAngle),
      maxAngle(_maxAngle),
      targetTimeMs(_targetTimeMs),
      autoDeactivationDelayMs(_autoDeactivationDelayMs)
{
  setMinMaxAngles(minAngle, maxAngle);

  if (targetTimeMs)
    setTargetTime(targetTimeMs);

  initLEDCforPWM();
}

/*********************************************************************
 * @brief 	class destructor
 * @param 	void
 * @return 	void
 *********************************************************************/
ESP32_Servo::~ESP32_Servo()
{
  ledcWrite(ledcChannel, 0);
  ledcDetachPin(servoPin); // detach sometimes leads to uncontrolled servo wobble

  usedLedcChannels[ledcChannel] = false; // free ledc channel
}

/*********************************************************************
 * @brief 	set GPIO for servo
 *          - only needed if GPIO not known at instantiation
 * @param 	servo pin / GPIO
 * @return 	void
 *********************************************************************/
void ESP32_Servo::setServoPin(uint8_t _servoPin)
{
  ledcWrite(ledcChannel, 0); // 0-duty-signal leads to a deactivated servo
  ledcDetachPin(servoPin);

  servoPin = _servoPin;

  ledcAttachPin(servoPin, ledcChannel); // Assignment of servo pin to PWM timer channel
}

/*********************************************************************
 * @brief 	initialization of ESPs ledc function unit to generate a PWM signal
 *          https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/api/ledc.html
 * @param 	PWM frequency to generate -> for standard servos usually 50Hz
 * @param   resolution (in bits) inside one duty/PWM cycle
 *          -> 1-20 bit for ESP32
 * @return  true : successfull configured
 *          false: error
 *********************************************************************/
bool ESP32_Servo::initLEDCforPWM(uint32_t _pwmFrequency, uint8_t _dutyCycleResolutionBits)
{
  if (ledcChannel == AUTOSELECT_LEDC_CHANNEL)
    autoselectLedcChannel();

  assert(ledcChannel > MAX_LEDC_CHANNELS || _dutyCycleResolutionBits > 8);

  auto frequency = ledcSetup(ledcChannel, _pwmFrequency, _dutyCycleResolutionBits);
  if (frequency == 0)
  {
    Serial.printf("%lu ms: ESP32_Servo::initLEDCforPWM: Error at ledcSetup \n", millis());
    return false;
  }

  ledcWrite(ledcChannel, 0);            // 0-duty-signal leads to a deactivated servo
  ledcAttachPin(servoPin, ledcChannel); // Assignment of servo pin to PWM timer channel

  return true;
}

/*********************************************************************
 * @brief 	returns next free LEDC channel
 *          - !! ONLY MANAGED INSIDE this library - not in general !!
 *          - the free channel is assigned to class member ledcChannel
 *            and is marked as used
 * @param 	false: starts occupying ledc channel from 0 to MAX_LEDC_CHANNELS
 *          true:  from MAX_LEDC_CHANNELS to 0
 * @return 	free LEDC channel
 *          255 on error
 *********************************************************************/
uint8_t ESP32_Servo::autoselectLedcChannel(bool _fromMaxToMin)
{
  int i = 0;
  int end = MAX_LEDC_CHANNELS;
  int increment = 1;

  if (_fromMaxToMin)
  {
    i = MAX_LEDC_CHANNELS;
    end = 0;
    increment = -1;
  }

  while (i != end)
  {
    if (usedLedcChannels[i] == false)
    {
      usedLedcChannels[i] = true;
      ledcChannel = i;
      return i;
    }

    i += increment;
  }

  return 255; // signals error
}

/*********************************************************************
 * @brief 	sets min and max angles as and stops for servo movements
 *          - if minAngle is equal to maxAngle -> diff is set to 1 or 2 degree
 *          - remark: timing values like targetTimeMs are not refreshed
 * @param 	min Angle in degree between SERVOANGLE_AT_MIN_PULSEWIDTH and SERVOANGLE_AT_MAX_PULSEWIDTH (usually 0 - 180)
 * @param 	max Angle in degree between SERVOANGLE_AT_MIN_PULSEWIDTH and SERVOANGLE_AT_MAX_PULSEWIDTH (usually 0 - 180)
 * @return 	void
 *********************************************************************/
void ESP32_Servo::setMinMaxAngles(uint8_t _minAngle, uint8_t _maxAngle)
{
  minAngle = _minAngle;
  maxAngle = _maxAngle;

  if (minAngle > maxAngle)
    std::swap(minAngle, maxAngle);

  if (minAngle < SERVOANGLE_AT_MIN_PULSEWIDTH)
    minAngle = SERVOANGLE_AT_MIN_PULSEWIDTH;

  if (maxAngle > SERVOANGLE_AT_MAX_PULSEWIDTH)
    maxAngle = SERVOANGLE_AT_MAX_PULSEWIDTH;

  if (minAngle == maxAngle)
  {
    if (minAngle != 0)
      minAngle -= 1;

    if (maxAngle != 180)
      maxAngle += 1;
  }

#if DEBUG_ESP32_SERVO
  Serial.printf("%lu ms: ESP32_Servo::setMinMaxAngles: minAngle= %d, maxAngle= %d \n", millis(), minAngle, maxAngle);
#endif
}

/*********************************************************************
 * @brief 	sets the desired target time for the speed controlled servo movement between minAngle and maxAngle
 *          - calcs delayMsPerDegreeFromTargetTime needed for speed control ticker
 *          - calcs anglespeedDegreePerSecond as inverse
 * @param 	desired target time in ms
 * @return 	realized target time - can differ due to rounding
 *********************************************************************/
uint16_t ESP32_Servo::setTargetTime(uint32_t _targetTimeMs)
{
  delayMsPerDegreeFromTargetTime = std::round(_targetTimeMs / (maxAngle - minAngle));

  anglespeedDegreePerSecond = ((double)(maxAngle - minAngle) / (double)(_targetTimeMs)) * 1000;

  if (!delayMsPerDegreeFromTargetTime)
    delayMsPerDegreeFromTargetTime = 1; // minimum value

  // store and return targetTime calculated from delay -> can be different due to rounding for ticker resolution (only ms)
  targetTimeMs = delayMsPerDegreeFromTargetTime * (maxAngle - minAngle);

#if DEBUG_ESP32_SERVO
  Serial.printf(
      "%lu ms: ESP32_Servo::setTargetTime: _targetTimeMs= %d, anglespeedDegreePerSecond= %d, delayMsPerDegreeFromTargetTime= %d, targetTimeMs after rounding= %d \n",
      millis(),
      _targetTimeMs,
      anglespeedDegreePerSecond,
      delayMsPerDegreeFromTargetTime,
      targetTimeMs);
#endif

  return targetTimeMs;
}

/*********************************************************************
 * @brief 	sets the servo speed for speed controlled movements by function moveToAngleWithSpeedControl()
 *          - only other characteristic value for the same functionality like setTargetTime()
 * @param 	angle speed in degree per second
 * @return 	realized angle speed - can differ due to rounding
 *********************************************************************/
uint16_t ESP32_Servo::setAnglespeedDegreePerSecond(uint16_t _anglespeedDegreePerSecond)
{
  auto _targetTimeMs = ((double)(maxAngle - minAngle) / (double)_anglespeedDegreePerSecond) * 1000.0;
  setTargetTime(_targetTimeMs);

#if DEBUG_ESP32_SERVO
  Serial.printf(
      "%lu ms: ESP32_Servo::setAnglespeedDegreePerSecond: _anglespeedDegreePerSecond= %d, anglespeedDegreePerSecond after recalc= %d \n",
      millis(),
      _anglespeedDegreePerSecond,
      anglespeedDegreePerSecond);
#endif

  return anglespeedDegreePerSecond; // is recalculated in setTargetTime()
}

/*********************************************************************
 * @brief 	activates the servo control
 *          by attaching PWM generation with LEDC to the servopin
 * @param 	void
 * @return 	void
 *********************************************************************/
void ESP32_Servo::activate()
{
  ledcWrite(ledcChannel, calcDutyForServoAngle(actualAngle)); // activate by setting duty to actualAngle which should be the last value before deactivation
  // ledcAttachPin(servoPin, ledcChannel);                       // Assignment of servo pin to PWM timer channel
  isActivated = true;

#if DEBUG_ESP32_SERVO
  Serial.printf("%lu ms: ESP32_Servo::activate \n", millis());
#endif
}

/*********************************************************************
 * @brief 	deactivates servo control
 *          by detaching PWM generation with LEDC from servopin
 *          -> usually a servo is deactivated and uses no more energy
 * @param 	void
 * @return 	void
 *********************************************************************/
void ESP32_Servo::deactivate()
{
  ledcWrite(ledcChannel, 0);
  // ledcDetachPin(servoPin);  //detach sometimes leads to uncontrolled servo wobble
  isActivated = false;

#if DEBUG_ESP32_SERVO
  Serial.printf("%lu ms: ESP32_Servo::deactivate \n", millis());
#endif
}

/*********************************************************************
 * @brief 	calculates the PWM/LEDC duty for a given servo angle
 * @param 	servo angle in degree
 *          -> must be inside the defined min max values -> is NOT CHECKED HERE
 * @return 	duty value
 *********************************************************************/
uint16_t ESP32_Servo::calcDutyForServoAngle(uint8_t _servoAngle)
{
  // Interpolation of the pulse width in µs belonging to the given angle
  //(double) to avoid rounding errors and because float cannot be used in ISR
  double servoPulseWidthMicrosec = SERVO_MIN_PULSEWIDTH +
                                   (double)(SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) /
                                       (double)(SERVOANGLE_AT_MAX_PULSEWIDTH - SERVOANGLE_AT_MIN_PULSEWIDTH) *
                                       (double)(_servoAngle - SERVOANGLE_AT_MIN_PULSEWIDTH);

  // Calculation of the number of "timer increments" (="ticks")
  uint16_t _duty = servoPulseWidthMicrosec *
                       ((double)LEDC_TIMER_RESOLUTION_STEPS / (double)SERVO_PWM_INTERVAL_MICROSEC) +
                   0.5; //+0.5 -> round up for double to int

#if DEBUG_ESP32_SERVO
  Serial.printf(
      "%lu ms: servoPulseWidthMicrosec= %f: _duty= %d \n",
      millis(),
      servoPulseWidthMicrosec,
      _duty);
#endif

  return _duty;
}

/*********************************************************************
 * @brief 	checks the given angle to minAngle and maxAngle
 * @param 	angle to check
 * @return 	angle inside min/max
 *********************************************************************/
uint8_t ESP32_Servo::checkforMinMaxAngle(uint8_t _angleToCheck)
{
  if (_angleToCheck > maxAngle)
    return maxAngle;
  else if (_angleToCheck < minAngle)
    return minAngle;

  return _angleToCheck;
}

/*********************************************************************
 * @brief 	moves servo to given target angle with full physical servo speed
 *          - activation is done automatic
 *          - this function only sets the PWM/LEDC values for the servo and is left
 *            -> usually the servo will continue to move after the end of the function, but the actualAngle is already set to targeAngle!!
 *          - the servo will be deactivated if SERVO_AUTO_DEACTIVATION_DELAY_MS is != 0
 *            -> if this time is too short, the servo will not reach its position!!
 * @param 	target angle in degree
 * @return 	void
 *********************************************************************/
void ESP32_Servo::moveToAngle(uint8_t _targetAngle)
{
  targetAngle = checkforMinMaxAngle(_targetAngle);

  if (isActivated)
  {
#if DEBUG_ESP32_SERVO
    Serial.printf("%lu ms: moveToAngle: isActivated \n", millis());
#endif
    tickerForAutoDeactivation.detach();
    tickerForSpeedControl.detach();
  }
  else
    activate();

  // write duty to LEDC-PWM-Generator
  ledcWrite(ledcChannel, calcDutyForServoAngle(targetAngle));

  actualAngle = targetAngle;

  if (autoDeactivationDelayMs)
  {
#if DEBUG_ESP32_SERVO
    Serial.printf("%lu ms: moveToAngle: autoDeactivationDelayMs= %d, tickerForAutoDeactivation ends at %d ms \n",
                  millis(), autoDeactivationDelayMs, millis() + autoDeactivationDelayMs);
#endif

    // tickerForAutoDeactivation.detach();
    tickerForAutoDeactivation.once_ms(
        autoDeactivationDelayMs,
        callbackTickerForAutoDeactivation,
        this);
  }
}

/*********************************************************************
 * @brief 	callback for deactivation ticker
 *          - must be a static function
 * @param 	pointer to belonging ESP32_Servo instance
 * @return 	void
 *********************************************************************/
void ESP32_Servo::callbackTickerForAutoDeactivation(ESP32_Servo *ptrServo)
{
  ptrServo->deactivate();
}

/*********************************************************************
 * @brief 	moves servo to given target angle with speed control
 *          - a ticker is started to control the movement in one degree steps
 *          - the function is left before movement and movement control ends
 *          - the servo will be deactivated after reaching its target position if SERVO_AUTO_DEACTIVATION_DELAY_MS is != 0
 * @param 	target angle in degree
 * @param 	delay in ms per degree
 *          standard value is DELAY_FROM_TARGET_TIME -> no specific value is needed after init
 * @return 	void
 *********************************************************************/
void ESP32_Servo::moveToAngleWithSpeedControl(uint8_t _targetAngle, uint16_t _delayMsPerDegree)
{
  targetAngle = checkforMinMaxAngle(_targetAngle);

  if (_delayMsPerDegree == DELAY_FROM_TARGET_TIME)
    _delayMsPerDegree = delayMsPerDegreeFromTargetTime;

  if (isActivated)
  {
#if DEBUG_ESP32_SERVO
    Serial.printf("%lu ms: moveToAngleWithSpeedControl: isActivated \n", millis());
#endif
    tickerForAutoDeactivation.detach();
    tickerForSpeedControl.detach();
  }
  else
    activate();

  // tickerForSpeedControl.detach();

  tickerForSpeedControl.attach_ms(
      _delayMsPerDegree,
      callbackTickerForSpeedControl,
      this);
}

/*********************************************************************
 * @brief 	callback for speed control ticker
 *          - each call moves the servo one degree to its target angle
 * @param 	pointer to belonging ESP32_Servo instance
 * @return 	void
 *********************************************************************/
void ESP32_Servo::callbackTickerForSpeedControl(ESP32_Servo *ptrServo)
{
  // end position reached:
  if (ptrServo->actualAngle == ptrServo->targetAngle)
  {
#if DEBUG_ESP32_SERVO
    Serial.printf("%lu ms: ESP32_Servo::callbackTickerForSpeedControl:  end position reached \n", millis());
#endif

    ptrServo->tickerForSpeedControl.detach();
    if (ptrServo->autoDeactivationDelayMs)
      ptrServo->deactivate();
    return;
  }

  // we have still to move:
  if (ptrServo->actualAngle <= ptrServo->targetAngle)
    ptrServo->actualAngle += 1;
  else
    ptrServo->actualAngle -= 1;

  ledcWrite(ptrServo->ledcChannel, ptrServo->calcDutyForServoAngle(ptrServo->actualAngle));
}
