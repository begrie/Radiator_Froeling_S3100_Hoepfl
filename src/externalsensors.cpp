#include "externalsensors.h"
#include "debug.h"

#define VENTILATOR_OFF false
#define VENTILATOR_ON true

#define AIR_INPUT_FLAP_CLOSED false
#define AIR_INPUT_FLAP_OPENED true

/*********************
 * STATIC DEFINITIONS
 *********************/
SemaphoreHandle_t radiator::ExternalSensors::semaphoreExternalSensors = NULL;
// std::stringstream radiator::ExternalSensors::bufferStringStream;
std::string radiator::ExternalSensors::messageBuf;

bool radiator::ExternalSensors::radiatorIsBurning;

int radiator::ExternalSensors::dhtGPIO = GPIO_FOR_DHT11;
DHT radiator::ExternalSensors::tempHumidityDHTSensor(GPIO_FOR_DHT11, DHTTYPE);
int16_t radiator::ExternalSensors::lastRoomTemperature = 0;
int16_t radiator::ExternalSensors::lastRoomHumidity = 0;

int radiator::ExternalSensors::ventilatorRelaisGPIO = GPIO_FOR_VENTILATOR_RELAIS;
bool radiator::ExternalSensors::ventilatorRelaisState = VENTILATOR_OFF;

int radiator::ExternalSensors::servoForAirInputFlapGPIO = GPIO_FOR_SERVO_FOR_AIR_INPUT_FLAP;
ESP32_Servo radiator::ExternalSensors::servoForAirInputFlap(GPIO_FOR_SERVO_FOR_AIR_INPUT_FLAP);
bool radiator::ExternalSensors::airInputFlapIsOpen = false;

int radiator::ExternalSensors::leakWaterSensorGPIO = GPIO_FOR_LEAKWATER_SENSOR;
volatile bool radiator::ExternalSensors::leakWaterDetected = false;

int radiator::ExternalSensors::acCurrentSensorGPIO = GPIO_FOR_AC_CURRENT_SENSOR;
Ticker radiator::ExternalSensors::tickerForReadAcCurrentSensor;
volatile int16_t radiator::ExternalSensors::acCurrentAnalogReadAmplitudeMillivolt = 0;

/*********************************************************************
 * @brief 	initialization of external sensors and actors(relais, servo)
 * @param 	GPIOs for the sensors/actors
 *          0 -> disable the specific sensor & functionality
 * @return 	void
 *********************************************************************/
bool radiator::ExternalSensors::initExternalSensors()
{
  semaphoreExternalSensors = xSemaphoreCreateMutex();
  messageBuf.reserve(2000);

  bool result = true;

  result &= initTempHumidityDHTSensor();

  result &= initVentilator();

  result &= initAirInputFlap();

  result &= initLeakWaterSensor();

  result &= initAcCurrentSensor();

  TaskHandle_t HandlexTaskExternalSensors;

  // Create RTOS task
  BaseType_t _Result = xTaskCreatePinnedToCore(
      xTaskExternalSensors,        // Task function
      "xTaskExternalSensors",      // String with name of task
      4096,                        // Stack size in bytes
      NULL,                        // Parameter passed as input of the task
      uxTaskPriorityGet(NULL),     // Priority of the task: higher values -> higher priority
                                   // with uxTaskPriorityGet(NULL)-> same priority as current task
      &HandlexTaskExternalSensors, // Task handle (Typ: TaskHandle_t)
      1);                          // Core 0 or 1 (Arduino code by default on Core 1)

  if (_Result != pdPASS)
  {
    throw std::runtime_error("xTaskExternalSensors: Error creating xTask");
  }

  return result;
}

/*********************************************************************
 * @brief 	get all actual sensor values as one string
 * @param 	void
 * @return 	sensor values in format
 *          [Heizungsraum-Temp] = [25°]
 *          [Heizungsraum-Feuchtigkeit] = [70%]"
 *          [Ventilator] = [ON]
 *          [Zuluftklappe] = [ZU]
 *          [Leckwassersensor] = [TROCKEN]
 *          [Stromstaerke] = [3.12A]
 *********************************************************************/
std::string radiator::ExternalSensors::getSensorValues()
{
  // xSemaphoreTake(semaphoreExternalSensors, portMAX_DELAY);
  // bufferStringStream.str(""); // empties content
  // bufferStringStream << "[Heizungsraum-Temp] = [" << lastRoomTemperature << "°] \n"
  //                    << "[Heizungsraum-Feuchtigkeit] = [" << lastRoomHumidity << "%] \n"
  //                    << "[Ventilator] = " << (ventilatorRelaisState ? "[AN]" : "[AUS]") << " \n"
  //                    << "[Zuluftklappe] = " << (airInputFlapIsOpen ? "[OFFEN]" : "[ZU]") << " \n"
  //                    << "[Leckwassersensor] = " << (leakWaterDetected ? "[!!!!! ALARM !!!!!]" : "[TROCKEN]") << " \n"
  //                    << "[Stromstaerke] = [" << std::fixed << std::setprecision(2) << getAcCurrentAmpereRMS() << "A] \n"
  // //                    << "[Stromstaerke] = [" << getAcCurrentAmpereRMS() << "A] \n"
  //                    << std::endl;
  // xSemaphoreGive(semaphoreExternalSensors);
  // return bufferStringStream.str();

  return "[Heizungsraum-Temp] = [" +
         std::to_string(lastRoomTemperature) + "°] \n" +
         "[Heizungsraum-Feuchtigkeit] = [" + std::to_string(lastRoomHumidity) + "%] \n" +
         "[Ventilator] = " + (std::string)(ventilatorRelaisState ? "[AN]" : "[AUS]") + (std::string) "\n" +
         "[Zuluftklappe] = " + (std::string)(airInputFlapIsOpen ? "[OFFEN]" : "[ZU]") + (std::string) "\n" +
         "[Leckwassersensor] = " + (std::string)(leakWaterDetected ? "[!!!!! ALARM !!!!!]" : "[TROCKEN]") + (std::string) "\n" +
         //  "[Stromstaerke] = [" + std::to_string(getAcCurrentAmpereRMS()) + "A] \n";
         "[Stromstaerke] = [" + getAcCurrentAmpereRMSasString() + "A] \n";
}

/*********************************************************************
 * @brief 	get all actual sensor values as one string in JSON format
 *          - only to add in output.cpp from formatValueDataAsJSON()
 *            -> NOT complete:  {}brackets are missing
 * @param 	void
 * @return 	sensor values in json format WITHOUT {} brackets:
 *          e.g.:   "Heizungsraum-Temp": "25°",
 *                  "Heizungsraum-Feuchtigkeit": "70%",
 *                  "Ventilator": "ON",
 *                  "Zuluftklappe": "ZU",
 *                  "Leckwassersensor": "TROCKEN",
 *                  "Stromstaerke": "3.12A",

 *********************************************************************/
std::string radiator::ExternalSensors::getSensorValuesForJSON()
{
  // xSemaphoreTake(semaphoreExternalSensors, portMAX_DELAY);
  // bufferStringStream.str(""); // empties content
  // bufferStringStream
  //     << "\"Heizungsraum-Temp\": \"" << lastRoomTemperature << "°\","
  //     << "\"Heizungsraum-Feuchtigkeit\": \"" << lastRoomHumidity << "%\","
  //     << "\"Ventilator\": " << (ventilatorRelaisState ? "\"AN\"," : "\"AUS\",")
  //     << "\"Zuluftklappe\": " << (airInputFlapIsOpen ? "\"OFFEN\"," : "\"ZU\",")
  //     << "\"Leckwassersensor\": " << (leakWaterDetected ? "\"!!!!! ALARM: LECKWASSER !!!!!\"," : "\"TROCKEN\",")
  //     << "\"Stromstaerke\": "<< std::fixed << std::setprecision(2) << getAcCurrentAmpereRMS() << "\"A\"," << std::endl;
  // xSemaphoreGive(semaphoreExternalSensors);
  // return bufferStringStream.str();

  return "\"Heizungsraum-Temp\": \"" + std::to_string(lastRoomTemperature) + "°\"," +
         "\"Heizungsraum-Feuchtigkeit\": \"" + std::to_string(lastRoomHumidity) + "%\"," +
         "\"Ventilator\": " + (std::string)(ventilatorRelaisState ? "\"AN\"," : "\"AUS\",") +
         "\"Zuluftklappe\": " + (std::string)(airInputFlapIsOpen ? "\"OFFEN\"," : "\"ZU\",") +
         "\"Leckwassersensor\": " + (std::string)(leakWaterDetected ? "\"!!!!! ALARM: LECKWASSER !!!!!\"," : "\"TROCKEN\",") +
         "\"Stromstaerke\": \"" + getAcCurrentAmpereRMSasString() + "A\"," +
         "\"Uptime\": \"" + std::to_string(millis() / (1000 * 60)) + "min\"," +
         "\"ESP.getFreeHeap()\": \"" + std::to_string(ESP.getFreeHeap()) + "bytes\"," +
         "\"ESP.getMinFreeHeap()\": \"" + std::to_string(ESP.getMinFreeHeap()) + "bytes\"," +
         "\"ESP.getMaxAllocHeap()\": \"" + std::to_string(ESP.getMaxAllocHeap()) + "bytes\",";
}

/*********************************************************************
 * @brief 	generates and return the header for CSV file output
 * @param 	void
 * @return 	delimiter separated header
 *          e.g.: Heizungsraum-Temp; Heizungsraum-Feuchtigkeit; Ventilator; Zuluftklappe; Leckwassersensor; Stromstaerke;
 *********************************************************************/
std::string radiator::ExternalSensors::getSensorValueHeaderForCSV()
{
  // bufferStringStream.str(""); // empties content

  // bufferStringStream << "Heizungsraum-Temp" << DELIMITER_FOR_CSV_FILE
  //                    << "Heizungsraum-Feuchtigkeit" << DELIMITER_FOR_CSV_FILE
  //                    << "Ventilator" << DELIMITER_FOR_CSV_FILE
  //                    << "Zuluftklappe" << DELIMITER_FOR_CSV_FILE
  //                    << "Leckwassersensor" << DELIMITER_FOR_CSV_FILE
  //                    << "Stromstaerke" << std::endl;
  // return bufferStringStream.str();

  return "Heizungsraum-Temp" DELIMITER_FOR_CSV_FILE
         "Heizungsraum-Feuchtigkeit" DELIMITER_FOR_CSV_FILE
         "Ventilator" DELIMITER_FOR_CSV_FILE
         "Zuluftklappe" DELIMITER_FOR_CSV_FILE
         "Leckwassersensor" DELIMITER_FOR_CSV_FILE
         "Stromstaerke" DELIMITER_FOR_CSV_FILE;
}

/*********************************************************************
 * @brief 	generates and return the values at this moment for CSV file output
 * @param 	void
 * @return 	delimiter separated values
 *          e.g.: 25°; 70%; AUS; OFFEN; TROCKEN; 3.12A;
 *********************************************************************/
std::string radiator::ExternalSensors::getSensorValueDataForCSV()
{
  // xSemaphoreTake(semaphoreExternalSensors, portMAX_DELAY);
  // bufferStringStream.str(""); // empties content
  // bufferStringStream << lastRoomTemperature << "°" << DELIMITER_FOR_CSV_FILE
  //                    << lastRoomHumidity << "%" << DELIMITER_FOR_CSV_FILE
  //                    << (ventilatorRelaisState ? "AN" : "AUS") << DELIMITER_FOR_CSV_FILE
  //                    << (airInputFlapIsOpen ? "OFFEN" : "ZU") << DELIMITER_FOR_CSV_FILE
  //                    << (leakWaterDetected ? "!!!!! ALARM !!!!!" : "TROCKEN") << DELIMITER_FOR_CSV_FILE
  //                    << getAcCurrentAmpereRMS() << std::fixed << std::setprecision(2) << "A" << DELIMITER_FOR_CSV_FILE << std::endl;
  // xSemaphoreGive(semaphoreExternalSensors);
  // return bufferStringStream.str();

  return std::to_string(lastRoomTemperature) + "°" DELIMITER_FOR_CSV_FILE +
         std::to_string(lastRoomHumidity) + "%" DELIMITER_FOR_CSV_FILE +
         (std::string)(ventilatorRelaisState ? "AN" : "AUS") + (std::string)DELIMITER_FOR_CSV_FILE +
         (std::string)(airInputFlapIsOpen ? "OFFEN" : "ZU") + (std::string)DELIMITER_FOR_CSV_FILE +
         (std::string)(leakWaterDetected ? "!!!!! ALARM !!!!!" : "TROCKEN") + (std::string)DELIMITER_FOR_CSV_FILE +
         //  std::to_string(getAcCurrentAmpereRMS()) + "A" DELIMITER_FOR_CSV_FILE;
         getAcCurrentAmpereRMSasString() + "A" DELIMITER_FOR_CSV_FILE;
}

/**********************************************************************
 *@brief   task for cyclic external sensors/actors actions
           runs endless
 *@param   pointer to parameter for the task -> nothing here!
 *@return  void
 **********************************************************************/
void radiator::ExternalSensors::xTaskExternalSensors(void *parameter)
{
  const uint16_t cycleTimeMs = 1000;
  ulong cycleTimestamp = 0;

  const int16_t DHTreadIntervallMs = 2500; // sensor produces a value only all 2 sec
  ulong DHTreadTimestamp = 0;

  bool buzzerIsRunning = false;

  while (true) // the xTask runs "endless"
  {
    cycleTimestamp = millis();

    if (dhtGPIO && // enabled?
        millis() >= DHTreadTimestamp + DHTreadIntervallMs)
    {
      xSemaphoreTake(semaphoreExternalSensors, portMAX_DELAY);

      RADIATOR_LOG_DEBUG(getMillisAndTime() << "start DHT reading" << std::endl;)
      lastRoomTemperature = readTemp(); // reading with DHT11 can last up to 275 ms!
      RADIATOR_LOG_DEBUG(getMillisAndTime() << "lastRoomTemperature=" << lastRoomTemperature << std::endl;)
      lastRoomHumidity = readHumidity(); // reading with DHT11 can last up to 275 ms
      RADIATOR_LOG_DEBUG(getMillisAndTime() << "lastRoomHumidity=" << lastRoomHumidity << std::endl;)
      DHTreadTimestamp = millis();

      xSemaphoreGive(semaphoreExternalSensors);
    }

    if (leakWaterSensorGPIO) // enabled?
    {
      if (leakWaterDetected) // leakWaterDetected is changed by ISR
      {
        // make continous noise
        pinMode(BUZZER_PIN, OUTPUT);
        digitalWrite(BUZZER_PIN, HIGH);
        buzzerIsRunning = true;

        messageBuf = getMillisAndTime() + "!!!!!!!! LEAK WATER DETECTED  !!!!!!!!";
        std::cout << messageBuf << std::endl;
        LOG_fatal << messageBuf << std::endl;
        radiator::NetworkHandler::publishToMQTT(messageBuf);
      }
      else if (buzzerIsRunning)
      {
        pinMode(BUZZER_PIN, OUTPUT);
        digitalWrite(BUZZER_PIN, LOW);
        buzzerIsRunning = false;

        messageBuf = getMillisAndTime() + "#### END leak water detected ####";
        std::cout << messageBuf << std::endl;
        LOG_fatal << messageBuf << std::endl;
        radiator::NetworkHandler::publishToMQTT(messageBuf);
      }
    }

    autoControlVentilatorAndFlap();

    // // for debug only:
    // static ulong lastOutput = 0;
    // if (millis() > lastOutput + MQTT_OUTPUTINTERVALL_SEC * 1000)
    // {
    //   lastOutput = millis();
    //   std::stringstream message;
    //   message << millis() << " ms: xTaskExternalSensors: radiatorIsBurning= " << radiatorIsBurning
    //           << ", lastRoomTemperature= " << lastRoomTemperature
    //           << ", lastRoomHumidity= " << lastRoomHumidity
    //           << ", ventilatorRelaisState= " << ventilatorRelaisState
    //           << ", airInputFlapIsOpen= " << airInputFlapIsOpen
    //           << ", leakWaterDetected= " << leakWaterDetected
    //           << ", acCurrentAnalogReadAmplitudeMillivolt= " << acCurrentAnalogReadAmplitudeMillivolt << ", acCurrentAmpere= " << getAcCurrentAmpereRMS()
    //           << std::endl;
    //   radiator::NetworkHandler::publishToMQTT(message.str(), "/externalsensors", 1, false);
    //   RADIATOR_LOG_DEBUG( message.str();
    // }
    // radiator::NetworkHandler::publishToMQTT("", "/externalsensors", 1, false);

    // static ulong nextLog = 0;
    // if (millis() >= nextLog)
    // {
    //   nextLog = millis() + 60000;
    //   DEBUG_STACK_HIGH_WATERMARK
    // }

    // try to hold the cycle time - independent from used working time
    int waitForMs = cycleTimestamp + cycleTimeMs - millis();
    if (waitForMs <= 0)
      waitForMs = 2;                      // give other tasks a minimum chance to work
    vTaskDelay(pdMS_TO_TICKS(waitForMs)); // to avoid overload of ESP
  }
}

/*********************************************************************
 * @brief 	Automatic control of ventilator and air input flap
 *          -> on base of the radiators running/fire burning state
 *             and the (from DHT sensor) measured temperature / humidity values
 * @param 	void
 * @return 	void
 *********************************************************************/
void radiator::ExternalSensors::autoControlVentilatorAndFlap()
{
  // control of air input flap
  if (servoForAirInputFlapGPIO) // GPIO defined for functionality enabling?
  {
    if (radiatorIsBurning && !airInputFlapIsOpen)
    {
      openAirInputFlap();
    }

    if (!radiatorIsBurning && airInputFlapIsOpen)
    {
      closeAirInputFlap();
    }
  }

  // control of ventilator
  if (ventilatorRelaisGPIO) // GPIO defined for functionality enabling?
  {
    if (lastRoomTemperature >= MIN_TEMP_FOR_VENTILATOR_ON &&
        lastRoomHumidity <= MAX_HUMIDITY_FOR_VENTILATOR_RUN &&
        !radiatorIsBurning) // ventilation only allowed when radiator is NOT UNDER FIRE
                            // !radiatorIsBurning && !airInputFlapIsOpen) // ventilation only allowed when radiator is not under fire and air input flap is closed
                            // TODO: now we are on the safe side -> check later if neccessary or if it's ok to vent during burning? ...
    {
      if (!ventilatorRelaisState)
        setVentilatorOn();
    }
    else if (ventilatorRelaisState &&                                                   // vent running?
             (radiatorIsBurning || lastRoomTemperature <= MAX_TEMP_FOR_VENTILATOR_OFF)) // turn off without time restrictions, because if radiator is burning -> turn off vent immediatelly
    {
      setVentilatorOff();
    }
  }
}

/*********************************************************************
 * @brief 	init temperature and humidity DHT11 sensor
 * @param 	void
 * @return 	void
 *********************************************************************/
bool radiator::ExternalSensors::initTempHumidityDHTSensor()
{
  if (!dhtGPIO) // GPIO defined for functionality enabling?
    return false;

  tempHumidityDHTSensor.begin();

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float humid = tempHumidityDHTSensor.readHumidity();
  float temp = tempHumidityDHTSensor.readTemperature(); // Read temperature as Celsius (the default)

  // Check if any reads failed and exit early (to try again).
  if (isnan(humid) || isnan(temp))
  {
    RADIATOR_LOG_ERROR(getMillisAndTime() << "initTempHumidityDHTSensor(): Failed to read from DHT sensor!" << std::endl;)
    return false;
  }

  RADIATOR_LOG_INFO(getMillisAndTime() << "initTempHumidityDHTSensor(): Humidity= " << humid << "%, Temperature= " << temp << "°C" << std::endl;)

  return true;
}

/*********************************************************************
 * @brief 	get temperature from dht sensor
 *          - Reading takes about 250 milliseconds!
 *          - Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
 * @param 	void
 * @return 	temperature in °C
 *          -> rounded as INTEGER !
 *********************************************************************/
int16_t radiator::ExternalSensors::readTemp()
{
  float temp = tempHumidityDHTSensor.readTemperature();

  static uint16_t readErrors = 0;

  if (isnan(temp)) // Check if reads failed and return "old" temperature
  {
    readErrors++;
    messageBuf = getMillisAndTime() + "readTemp: Failed to read temperature from DHT sensor (" + std::to_string(readErrors) + ")";
    RADIATOR_LOG_INFO(messageBuf << std::endl;)

    if (readErrors >= 50)
    {
      RADIATOR_LOG_ERROR(messageBuf << std::endl;)
      radiator::NetworkHandler::publishToMQTT(messageBuf, MQTT_SUBTOPIC_SYSLOG);
      readErrors = 0; // only to avoid too many messages
    }
    return lastRoomTemperature;
  }

  readErrors = 0;
  return (int16_t)temp + 0.5; //  float value rounded
}

/*********************************************************************
 * @brief 	get humidity from dht sensor
 *          - Reading takes about 250 milliseconds!
 *          - Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
 * @param 	void
 * @return 	humidity in %
 *          -> rounded as INTEGER !
 *********************************************************************/
int16_t radiator::ExternalSensors::readHumidity()
{
  float humid = tempHumidityDHTSensor.readHumidity();

  static uint16_t readErrors = 0;

  if (isnan(humid)) // Check if reads failed and return "old" humidity
  {
    readErrors++;
    messageBuf = getMillisAndTime() + "readHumidity: Failed to read humidity from DHT sensor (" + std::to_string(readErrors) + ")";
    RADIATOR_LOG_INFO(messageBuf << std::endl;)

    if (readErrors >= 100)
    {
      RADIATOR_LOG_ERROR(messageBuf << std::endl;)
      radiator::NetworkHandler::publishToMQTT(messageBuf, MQTT_SUBTOPIC_SYSLOG);
      readErrors = 0; // only to avoid too many messages
    }
    return lastRoomHumidity;
  }

  readErrors = 0;
  return (int16_t)humid + 0.5; // float value rounded
}

/*********************************************************************
 * @brief 	init ventilator
 * @param 	void
 * @return 	void
 *********************************************************************/
bool radiator::ExternalSensors::initVentilator()
{
  if (!ventilatorRelaisGPIO) // GPIO defined for functionality enabling?
    return false;

  pinMode(ventilatorRelaisGPIO, OUTPUT);

  setVentilatorOff();

  return true;
}

/*********************************************************************
 * @brief 	switch ventilator ON
 * @param 	void
 * @return 	void
 *********************************************************************/
void radiator::ExternalSensors::setVentilatorOn()
{
  xSemaphoreTake(semaphoreExternalSensors, portMAX_DELAY);
  digitalWrite(ventilatorRelaisGPIO, VENTILATOR_ON);

  ventilatorRelaisState = VENTILATOR_ON;
  xSemaphoreGive(semaphoreExternalSensors);

  RADIATOR_LOG_INFO(getMillisAndTime() << "Ventilator ON" << std::endl;)
}

/*********************************************************************
 * @brief 	switch ventilator OFF
 * @param 	void
 * @return 	void
 *********************************************************************/
void radiator::ExternalSensors::setVentilatorOff()
{
  xSemaphoreTake(semaphoreExternalSensors, portMAX_DELAY);
  digitalWrite(ventilatorRelaisGPIO, VENTILATOR_OFF);

  ventilatorRelaisState = VENTILATOR_OFF;
  xSemaphoreGive(semaphoreExternalSensors);

  RADIATOR_LOG_INFO(getMillisAndTime() << "Ventilator OFF" << std::endl;)
}

/*********************************************************************
 * @brief 	init air input flap
 * @param 	GPIO for flap servo
 *          0: deactivate flap and functionality
 * @return 	void
 *********************************************************************/
bool radiator::ExternalSensors::initAirInputFlap()
{
  if (!servoForAirInputFlapGPIO) // GPIO defined for functionality enabling?
    return false;

  servoForAirInputFlap.setMinMaxAngles(CLOSED_ANGLE_FOR_SERVO_FOR_AIR_INPUT_FLAP, OPENED_ANGLE_FOR_SERVO_FOR_AIR_INPUT_FLAP);
  servoForAirInputFlap.setAnglespeedDegreePerSecond(DEGREE_PER_SECOND_FOR_SERVO_FOR_AIR_INPUT_FLAP);

  openAirInputFlap();

  return true;
}

/*********************************************************************
 * @brief 	Open the air input flap with the connected servo
 * @param 	void
 * @return 	void
 *********************************************************************/
void radiator::ExternalSensors::openAirInputFlap()
{
  xSemaphoreTake(semaphoreExternalSensors, portMAX_DELAY);
  servoForAirInputFlap.moveToAngleWithSpeedControl(OPENED_ANGLE_FOR_SERVO_FOR_AIR_INPUT_FLAP); // runs asynchronous in background ...

  airInputFlapIsOpen = AIR_INPUT_FLAP_OPENED; //... but state is set now
  xSemaphoreGive(semaphoreExternalSensors);

  RADIATOR_LOG_INFO(getMillisAndTime() << "Air Input Flap OPENED" << std::endl;)
}

/*********************************************************************
 * @brief 	Close the air input flap with the connected servo
 * @param 	void
 * @return 	void
 *********************************************************************/
void radiator::ExternalSensors::closeAirInputFlap()
{
  xSemaphoreTake(semaphoreExternalSensors, portMAX_DELAY);
  servoForAirInputFlap.moveToAngleWithSpeedControl(CLOSED_ANGLE_FOR_SERVO_FOR_AIR_INPUT_FLAP); // runs asynchronous in background ...

  airInputFlapIsOpen = AIR_INPUT_FLAP_CLOSED; //... but state is set now
  xSemaphoreGive(semaphoreExternalSensors);

  RADIATOR_LOG_INFO(getMillisAndTime() << "Air Input Flap CLOSED" << std::endl;)
}

/*********************************************************************
 * @brief 	init leak water sensor
 * @param 	GPIO for sensor
 *          0: deactivate sensor and functionality
 * @return 	void
 *********************************************************************/
bool radiator::ExternalSensors::initLeakWaterSensor()
{
  if (!leakWaterSensorGPIO) // GPIO defined for functionality enabling?
    return false;

  pinMode(leakWaterSensorGPIO, INPUT); // for GPIO 39 NO PULLUP available
  // pinMode(leakWaterSensorGPIO, INPUT_PULLUP);

  leakWaterDetected = !digitalRead(leakWaterSensorGPIO); // Raindrop sensor gives HIGH at NO WATER
  RADIATOR_LOG_INFO(getMillisAndTime() << "initLeakWaterSensor(): leakWaterDetected= " << leakWaterDetected << std::endl;)

  attachInterrupt(
      leakWaterSensorGPIO,
      []() IRAM_ATTR
      {
        leakWaterDetected = !digitalRead(leakWaterSensorGPIO); // Raindrop sensor gives HIGH at NO WATER
      },
      CHANGE);

  return true;
}

/*********************************************************************
 * @brief 	init AC current sensor
 * @param 	GPIO for sensor
 *          0: deactivate sensor and functionality
 * @return 	void
 *********************************************************************/
bool radiator::ExternalSensors::initAcCurrentSensor()
{
  if (!acCurrentSensorGPIO) // GPIO defined for functionality enabling?
    return false;

  auto readValue = analogReadMilliVolts(acCurrentSensorGPIO);

  // ticker executes every 2ms a function which reads the current sensor, determines the max. value of one 20ms/50Hz period and average the values every one second
  tickerForReadAcCurrentSensor.attach_ms(2, tickerCallbackForReadAndAverageAcCurrentSensor);

  return true;
}

/*********************************************************************
 * @brief 	function is used by a periodic ticker (2ms) to sample and evaluate the AC current signal:
 *          - the AC current sensor is read
 *          - the max and min values of one 20ms/50Hz ac period are determined
 *          - the values of 1 second (50 values) are averaged
 *          - the current is calculated with (max - min) / 2
 *            and stored in class member acCurrentAnalogReadAmplitudeMillivolt
 * @param 	void
 * @return 	void
 *********************************************************************/
void IRAM_ATTR radiator::ExternalSensors::tickerCallbackForReadAndAverageAcCurrentSensor()
{
  const uint16_t periodsForAveraging = 50; // 1 seconds for net frequency of 50Hz
  static ulong timestamp = 0;

  static uint16_t minVal = UINT16_MAX;
  static uint16_t maxVal = 0;

  static uint16_t minValArray[periodsForAveraging]{0};
  static uint16_t maxValArray[periodsForAveraging]{0};

  static uint16_t periodCounter = 0;

  // sample signal with ticker frequency
  auto val = analogReadMilliVolts(acCurrentSensorGPIO);

  // determine max and min from one period
  if (val < minVal)
    minVal = val;
  if (val > maxVal)
    maxVal = val;

  // save max and min from one period in valArray
  if (millis() - timestamp >= 20) // 20ms -> one period for 50Hz mains frequency
  {
    timestamp = millis();
    minValArray[periodCounter] = minVal;
    maxValArray[periodCounter] = maxVal;
    periodCounter++;
    minVal = UINT16_MAX;
    maxVal = 0;

    // calculate average for 1 second
    if (periodCounter >= periodsForAveraging)
    {
      int minSum = 0;
      int maxSum = 0;

      for (int i = 0; i < periodsForAveraging; i++)
      {
        minSum += minValArray[i];
        maxSum += maxValArray[i];
      }

      xSemaphoreTake(semaphoreExternalSensors, portMAX_DELAY);
      // desired value is half of the peak to peak value
      acCurrentAnalogReadAmplitudeMillivolt = (maxSum - minSum) / (periodsForAveraging * 2);
      xSemaphoreGive(semaphoreExternalSensors);

      periodCounter = 0;

      // // Version with filtering by ellimination of biggest and smallest values -> from testing: only small effects
      // int minSum = 0;
      // int maxSum = 0;

      // // sort to eliminate a number of biggest and smallest values for filtering
      // std::sort(minValArray, minValArray + periodsForAveraging);
      // std::sort(maxValArray, maxValArray + periodsForAveraging);

      // const uint8_t maxAndMinToEliminate = periodsForAveraging / 10;

      // // sum up without biggest and smallest values
      // for (int i = 0; i < periodsForAveraging; i++)
      // {
      //   if (i >= maxAndMinToEliminate)
      //     minSum += minValArray[i];
      //   if (i < periodsForAveraging - maxAndMinToEliminate)
      //     maxSum += maxValArray[i];
      // }

      // // desired value is half of the amplitude
      // acCurrentAnalogReadAmplitudeMillivolt = (maxSum - minSum) / ((periodsForAveraging - maxAndMinToEliminate * 2) * 2);
      // Serial.println(acCurrentAnalogReadAmplitudeMillivolt);

      // periodCounter = 0;
    }
  }
}

/*********************************************************************
 * @brief 	returns current in ampere
 *          -> calculated from member var acCurrentAnalogReadAmplitudeMillivolt
 * @param 	void
 * @return 	rms current in ampere
 *          -> rounded to 2 decimal places
 *********************************************************************/
double radiator::ExternalSensors::getAcCurrentAmpereRMS()
{
  auto peakCurrent = (double)(acCurrentAnalogReadAmplitudeMillivolt) * (double)AC_CURRENT_SENSOR_SCALE_AMPERE_PER_MILLIVOLT;
  auto rmsAmpere = peakCurrent * 0.70710678118;
  return (int)(rmsAmpere * 100 + 0.5) / 100.0; // rounded to 2 decimal places

  // return peakCurrent * 0.70710678118; // calculate rms from amplitude
  // return peakCurrent / 1.41421356;
}

/*********************************************************************
 * @brief 	returns current in ampere as std::string
 *          -> calculated from member var acCurrentAnalogReadAmplitudeMillivolt
 * @param 	void
 * @return 	string with rms current in ampere
 *          -> rounded to 2 decimal places
 *********************************************************************/
std::string radiator::ExternalSensors::getAcCurrentAmpereRMSasString()
{
  messageBuf = std::to_string(getAcCurrentAmpereRMS()); // delivers 6 digits
  return messageBuf.substr(0, messageBuf.length() - 4); // remove last 4
}
