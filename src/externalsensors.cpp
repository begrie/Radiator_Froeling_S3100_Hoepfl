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
  // std::string sensorValuesString;

  // xSemaphoreTake(semaphoreExternalSensors, portMAX_DELAY);
  // bufferStringStream.str(""); // empties content

  // bufferStringStream << "[Heizungsraum-Temp] = [" << lastRoomTemperature << "°] \n"
  //                    << "[Heizungsraum-Feuchtigkeit] = [" << lastRoomHumidity << "%] \n"
  //                    << "[Ventilator] = " << (ventilatorRelaisState ? "[AN]" : "[AUS]") << " \n"
  //                    << "[Zuluftklappe] = " << (airInputFlapIsOpen ? "[OFFEN]" : "[ZU]") << " \n"
  //                    << "[Leckwassersensor] = " << (leakWaterDetected ? "[!!!!! ALARM !!!!!]" : "[TROCKEN]") << " \n"
  //                    << "[Stromstaerke] = [" << getAcCurrentAmpereRMS() << "A] \n"
  //                    << std::endl;

  // xSemaphoreGive(semaphoreExternalSensors);

  // return bufferStringStream.str();
  // //  return bufferStringStream.rdbuf()->str();

  //   sensorValuesString = "[Heizungsraum-Temp] = [" + std::to_string(lastRoomTemperature) + "°]" + "\n";
  //   sensorValuesString += "[Heizungsraum-Feuchtigkeit] = [" + std::to_string(lastRoomHumidity) + "%]" + "\n";
  //   sensorValuesString += "[Ventilator] = " + ventilatorRelaisState ? "[AN]" : "[AUS]" + (std::string) "\n";
  //   sensorValuesString += "[Zuluftklappe] = " + airInputFlapIsOpen ? "[OFFEN]" : "[ZU]" + (std::string) "\n";
  //   sensorValuesString += "[Leckwassersensor] = " + leakWaterDetected ? "[!!!!! ALARM !!!!!]" : "[TROCKEN]" + (std::string) "\n";
  //   sensorValuesString += "[Stromstaerke] = [" + std::to_string(getAcCurrentAmpereRMS()) + "A]" + "\n";
  //   xSemaphoreGive(semaphoreExternalSensors);

  //   return sensorValuesString;

  return "[Heizungsraum-Temp] = [" + std::to_string(lastRoomTemperature) + "°]" + (std::string) "\n" +
         "[Heizungsraum-Feuchtigkeit] = [" + std::to_string(lastRoomHumidity) + "%]" + (std::string) "\n" +
         "[Ventilator] = " + (std::string)(ventilatorRelaisState ? "[AN]" : "[AUS]") + (std::string) "\n" +
         "[Zuluftklappe] = " + (std::string)(airInputFlapIsOpen ? "[OFFEN]" : "[ZU]") + (std::string) "\n" +
         "[Leckwassersensor] = " + (std::string)(leakWaterDetected ? "[!!!!! ALARM !!!!!]" : "[TROCKEN]") + (std::string) "\n" +
         "[Stromstaerke] = [" + std::to_string(getAcCurrentAmpereRMS()) + "A]" + (std::string) "\n";
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
std::string radiator::ExternalSensors::getSensorValuesAsJSON()
{
  // std::stringstream sensorValuesString;

  // xSemaphoreTake(semaphoreExternalSensors, portMAX_DELAY);
  // bufferStringStream.str(""); // empties content

  // bufferStringStream
  //     << "\"Heizungsraum-Temp\": \"" << lastRoomTemperature << "°\","
  //     << "\"Heizungsraum-Feuchtigkeit\": \"" << lastRoomHumidity << "%\","
  //     << "\"Ventilator\": " << (ventilatorRelaisState ? "\"AN\"," : "\"AUS\",")
  //     << "\"Zuluftklappe\": " << (airInputFlapIsOpen ? "\"OFFEN\"," : "\"ZU\",")
  //     << "\"Leckwassersensor\": " << (leakWaterDetected ? "\"!!!!! ALARM: LECKWASSER !!!!!\"," : "\"TROCKEN\",")
  //     << "\"Stromstaerke\": " << getAcCurrentAmpereRMS() << "\"A\"," << std::endl;

  // xSemaphoreGive(semaphoreExternalSensors);

  // return bufferStringStream.str();

  // std::string sensorValuesString;

  // sensorValuesString = "\"Heizungsraum-Temp\": \"" + std::to_string(lastRoomTemperature) + "°\",";
  // sensorValuesString += "\"Heizungsraum-Feuchtigkeit\": \"" + std::to_string(lastRoomHumidity) + "%\",";
  // sensorValuesString += "\"Ventilator\": " + (std::string)(ventilatorRelaisState ? "\"AN\"," : "\"AUS\",");
  // sensorValuesString += "\"Zuluftklappe\": " + (std::string)(airInputFlapIsOpen ? "\"OFFEN\"," : "\"ZU\",");
  // sensorValuesString += "\"Leckwassersensor\": " + (std::string)(leakWaterDetected ? "\"!!!!! ALARM: LECKWASSER !!!!!\"," : "\"TROCKEN\",");
  // sensorValuesString += "\"Stromstaerke\": " + std::to_string(getAcCurrentAmpereRMS()) + "\"A\",";

  // return sensorValuesString;

  return "\"Heizungsraum-Temp\": \"" + std::to_string(lastRoomTemperature) + "°\"," +
         "\"Heizungsraum-Feuchtigkeit\": \"" + std::to_string(lastRoomHumidity) + "%\"," +
         "\"Ventilator\": " + (std::string)(ventilatorRelaisState ? "\"AN\"," : "\"AUS\",") +
         "\"Zuluftklappe\": " + (std::string)(airInputFlapIsOpen ? "\"OFFEN\"," : "\"ZU\",") +
         "\"Leckwassersensor\": " + (std::string)(leakWaterDetected ? "\"!!!!! ALARM: LECKWASSER !!!!!\"," : "\"TROCKEN\",") +
         "\"Stromstaerke\": " + std::to_string(getAcCurrentAmpereRMS()) + "\"A\",";
}

/*********************************************************************
 * @brief 	generates and return the header for CSV file output
 * @param 	void
 * @return 	delimiter separated header
 *          e.g.: Heizungsraum-Temp; Heizungsraum-Feuchtigkeit; Ventilator; Zuluftklappe; Leckwassersensor; Stromstaerke;
 *********************************************************************/
std::string radiator::ExternalSensors::getSensorValueHeaderForCSV()
{
  // // std::stringstream valHeaderStr;
  // bufferStringStream.str(""); // empties content

  // bufferStringStream << "Heizungsraum-Temp" << DELIMITER_FOR_CSV_FILE
  //                    << "Heizungsraum-Feuchtigkeit" << DELIMITER_FOR_CSV_FILE
  //                    << "Ventilator" << DELIMITER_FOR_CSV_FILE
  //                    << "Zuluftklappe" << DELIMITER_FOR_CSV_FILE
  //                    << "Leckwassersensor" << DELIMITER_FOR_CSV_FILE
  //                    << "Stromstaerke" << std::endl;

  // return bufferStringStream.str();

  // std::string valHeaderStr;

  // valHeaderStr += "Heizungsraum-Temp" + (std::string)DELIMITER_FOR_CSV_FILE +
  //                 "Heizungsraum-Feuchtigkeit" + (std::string)DELIMITER_FOR_CSV_FILE +
  //                 "Ventilator" + (std::string)DELIMITER_FOR_CSV_FILE +
  //                 "Zuluftklappe" + (std::string)DELIMITER_FOR_CSV_FILE +
  //                 "Leckwassersensor" + (std::string)DELIMITER_FOR_CSV_FILE +
  //                 "Stromstaerke";

  // return valHeaderStr;

  return "Heizungsraum-Temp" + (std::string)DELIMITER_FOR_CSV_FILE +
         "Heizungsraum-Feuchtigkeit" + (std::string)DELIMITER_FOR_CSV_FILE +
         "Ventilator" + (std::string)DELIMITER_FOR_CSV_FILE +
         "Zuluftklappe" + (std::string)DELIMITER_FOR_CSV_FILE +
         "Leckwassersensor" + (std::string)DELIMITER_FOR_CSV_FILE +
         "Stromstaerke";
}

/*********************************************************************
 * @brief 	generates and return the values at this moment for CSV file output
 * @param 	void
 * @return 	delimiter separated values
 *          e.g.: 25°; 70%; AUS; OFFEN; TROCKEN; 3.12A;
 *********************************************************************/
std::string radiator::ExternalSensors::getSensorValueDataForCSV()
{
  // std::stringstream valStr;

  // xSemaphoreTake(semaphoreExternalSensors, portMAX_DELAY);
  // bufferStringStream.str(""); // empties content

  // bufferStringStream << lastRoomTemperature << "°" << DELIMITER_FOR_CSV_FILE
  //                    << lastRoomHumidity << "%" << DELIMITER_FOR_CSV_FILE
  //                    << (ventilatorRelaisState ? "AN" : "AUS") << DELIMITER_FOR_CSV_FILE
  //                    << (airInputFlapIsOpen ? "OFFEN" : "ZU") << DELIMITER_FOR_CSV_FILE

  //                    << (leakWaterDetected ? "!!!!! ALARM !!!!!" : "TROCKEN") << DELIMITER_FOR_CSV_FILE

  //                    << getAcCurrentAmpereRMS() << "A" << DELIMITER_FOR_CSV_FILE << std::endl;

  // xSemaphoreGive(semaphoreExternalSensors);
  // return bufferStringStream.str();

  // std::string valStr;

  // xSemaphoreTake(semaphoreExternalSensors, portMAX_DELAY);

  // valStr = std::to_string(lastRoomTemperature) + "°" + (std::string)DELIMITER_FOR_CSV_FILE;
  // valStr += std::to_string(lastRoomHumidity) + "%" + (std::string)DELIMITER_FOR_CSV_FILE;
  // valStr += ventilatorRelaisState ? "AN" : "AUS" + (std::string)DELIMITER_FOR_CSV_FILE;
  // valStr += airInputFlapIsOpen ? "OFFEN" : "ZU" + (std::string)DELIMITER_FOR_CSV_FILE;
  // valStr += leakWaterDetected ? "!!!!! ALARM !!!!!" : "TROCKEN" + (std::string)DELIMITER_FOR_CSV_FILE;
  // valStr += std::to_string(getAcCurrentAmpereRMS()) + "A" + (std::string)DELIMITER_FOR_CSV_FILE;

  // xSemaphoreGive(semaphoreExternalSensors);
  // return valStr;

  return std::to_string(lastRoomTemperature) + "°" + (std::string)DELIMITER_FOR_CSV_FILE +
         std::to_string(lastRoomHumidity) + "%" + (std::string)DELIMITER_FOR_CSV_FILE +
         (std::string)(ventilatorRelaisState ? "AN" : "AUS") + (std::string)DELIMITER_FOR_CSV_FILE +
         (std::string)(airInputFlapIsOpen ? "OFFEN" : "ZU") + (std::string)DELIMITER_FOR_CSV_FILE +
         (std::string)(leakWaterDetected ? "!!!!! ALARM !!!!!" : "TROCKEN") + (std::string)DELIMITER_FOR_CSV_FILE +
         std::to_string(getAcCurrentAmpereRMS()) + "A" + (std::string)DELIMITER_FOR_CSV_FILE;
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

    //    xSemaphoreTake(semaphoreExternalSensors, portMAX_DELAY);

    if (dhtGPIO && // enabled?
        millis() >= DHTreadTimestamp + DHTreadIntervallMs)
    {
      xSemaphoreTake(semaphoreExternalSensors, portMAX_DELAY);

      LOG_debug << millis() << " ms: start DHT reading" << std::endl;
      lastRoomTemperature = readTemp(); // reading with DHT11 can last up to 275 ms!
      LOG_debug << millis() << " ms: lastRoomTemperature=" << lastRoomTemperature << std::endl;
      lastRoomHumidity = readHumidity(); // reading with DHT11 can last up to 275 ms
      LOG_debug << millis() << " ms: lastRoomHumidity=" << lastRoomHumidity << std::endl;
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

        // std::string message = std::to_string(millis()) + " ms: !!!!!!!! LEAK WATER DETECTED  !!!!!!!!";
        messageBuf = std::to_string(millis()) + " ms: !!!!!!!! LEAK WATER DETECTED  !!!!!!!!";
        std::cout << messageBuf << std::endl;
        LOG_fatal << messageBuf << std::endl;
        radiator::NetworkHandler::publishToMQTT(messageBuf);
        // bufferStringStream.str(""); // empties content
        // bufferStringStream << millis() << " ms: !!!!!!!! LEAK WATER DETECTED  !!!!!!!!";
        // std::cout << bufferStringStream.str() << std::endl;
        // LOG_fatal << bufferStringStream.str() << std::endl;
        // radiator::NetworkHandler::publishToMQTT(bufferStringStream.str());
      }
      else if (buzzerIsRunning)
      {
        pinMode(BUZZER_PIN, OUTPUT);
        digitalWrite(BUZZER_PIN, LOW);
        buzzerIsRunning = false;

        // std::string message = std::to_string(millis()) + " ms: #### END leak water detected ####";
        messageBuf = std::to_string(millis()) + " ms: #### END leak water detected ####";
        std::cout << messageBuf << std::endl;
        LOG_fatal << messageBuf << std::endl;
        radiator::NetworkHandler::publishToMQTT(messageBuf);
        // bufferStringStream.str(""); // empties content
        // bufferStringStream << millis() << " ms: #### END leak water detected ####";
        // std::cout << bufferStringStream.str() << std::endl;
        // LOG_fatal << bufferStringStream.str() << std::endl;
        // radiator::NetworkHandler::publishToMQTT(bufferStringStream.str());
      }
    }

    autoControlVentilatorAndFlap();

    // static ulong lastOutput = 0;
    // if (millis() > lastOutput + MQTT_OUTPUTINTERVALL_SEC * 1000)
    // {
    //   lastOutput = millis();
    //   std::stringstream message;

    //   message << millis() << " ms: xTaskExternalSensors: radiatorIsBurning= " << radiatorIsBurning << ", lastRoomTemperature= " << lastRoomTemperature << ", lastRoomHumidity= " << lastRoomHumidity << ", ventilatorRelaisState= " << ventilatorRelaisState << ", airInputFlapIsOpen= " << airInputFlapIsOpen << ", leakWaterDetected= " << leakWaterDetected << ", acCurrentAnalogReadAmplitudeMillivolt= " << acCurrentAnalogReadAmplitudeMillivolt << ", acCurrentAmpere= " << getAcCurrentAmpereRMS() << std::endl;

    //   radiator::NetworkHandler::publishToMQTT(message.str(), "/externalsensors", 1, false);
    //   LOG_debug << message.str();
    // }
    // radiator::NetworkHandler::publishToMQTT("", "/externalsensors", 1, false);

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
        !radiatorIsBurning) // ventilation only allowed when radiator is not under fire
                            // !radiatorIsBurning && !airInputFlapIsOpen) // ventilation only allowed when radiator is not under fire and air input flap is closed
                            // TODO: now we are on the safe side -> check later if neccessary or if it's ok to vent during burning? ...
    {
      if (!ventilatorRelaisState)
      {
        setVentilatorOn();
      }
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
  // Read temperature as Celsius (the default)
  float temp = tempHumidityDHTSensor.readTemperature();

  // Check if any reads failed and exit early (to try again).
  if (isnan(humid) || isnan(temp))
  {
    LOG_error << millis() << " ms: initTempHumidityDHTSensor(): Failed to read from DHT sensor!" << std::endl;
    return false;
  }

  LOG_info << millis() << " ms: initTempHumidityDHTSensor(): Humidity= " << humid << "%, Temperature= " << temp << "°C" << std::endl;

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
  // return (int16_t)(tempHumidityDHTSensor.readTemperature() + 0.5); //  float value rounded

  float temp = tempHumidityDHTSensor.readTemperature();

  static uint16_t readErrors = 0;

  if (isnan(temp)) // Check if reads failed and return "old" temperature
  {
    readErrors++;
    // std::stringstream message;
    messageBuf = std::to_string(millis()) + " ms: readTemp(): Failed to read temperature from DHT sensor (" + std::to_string(readErrors) + ")";
    LOG_info << messageBuf;

    if (readErrors >= 100)
    {
      LOG_error << messageBuf;
      radiator::NetworkHandler::publishToMQTT(messageBuf, MQTT_TOPIC_SYSLOG);
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
  // return (int16_t)(tempHumidityDHTSensor.readHumidity() + 0.5); // rounded float value

  float humid = tempHumidityDHTSensor.readHumidity();

  static uint16_t readErrors = 0;

  if (isnan(humid)) // Check if reads failed and return "old" humidity
  {
    readErrors++;
    // std::stringstream message;
    messageBuf = std::to_string(millis()) + " ms: readHumidity(): Failed to read humidity from DHT sensor (" + std::to_string(readErrors) + ")";
    LOG_info << messageBuf;

    if (readErrors >= 100)
    {
      LOG_error << messageBuf;
      radiator::NetworkHandler::publishToMQTT(messageBuf, MQTT_TOPIC_SYSLOG);
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
  digitalWrite(ventilatorRelaisGPIO, VENTILATOR_ON);

  xSemaphoreTake(semaphoreExternalSensors, portMAX_DELAY);
  ventilatorRelaisState = VENTILATOR_ON;
  xSemaphoreGive(semaphoreExternalSensors);

  LOG_info << millis() << " ms: Ventilator ON" << std::endl;
}

/*********************************************************************
 * @brief 	switch ventilator OFF
 * @param 	void
 * @return 	void
 *********************************************************************/
void radiator::ExternalSensors::setVentilatorOff()
{
  digitalWrite(ventilatorRelaisGPIO, VENTILATOR_OFF);

  xSemaphoreTake(semaphoreExternalSensors, portMAX_DELAY);
  ventilatorRelaisState = VENTILATOR_OFF;
  xSemaphoreGive(semaphoreExternalSensors);

  LOG_info << millis() << " ms: Ventilator OFF" << std::endl;
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
  servoForAirInputFlap.moveToAngleWithSpeedControl(OPENED_ANGLE_FOR_SERVO_FOR_AIR_INPUT_FLAP); // runs asynchronous in background ...

  xSemaphoreTake(semaphoreExternalSensors, portMAX_DELAY);
  airInputFlapIsOpen = AIR_INPUT_FLAP_OPENED; //... but state is set now
  xSemaphoreGive(semaphoreExternalSensors);

  LOG_info << millis() << " ms: Air Input Flap OPENED" << std::endl;
}

/*********************************************************************
 * @brief 	Close the air input flap with the connected servo
 * @param 	void
 * @return 	void
 *********************************************************************/
void radiator::ExternalSensors::closeAirInputFlap()
{
  servoForAirInputFlap.moveToAngleWithSpeedControl(CLOSED_ANGLE_FOR_SERVO_FOR_AIR_INPUT_FLAP); // runs asynchronous in background ...

  xSemaphoreTake(semaphoreExternalSensors, portMAX_DELAY);
  airInputFlapIsOpen = AIR_INPUT_FLAP_CLOSED; //... but state is set now
  xSemaphoreGive(semaphoreExternalSensors);

  LOG_info << millis() << " ms: Air Input Flap CLOSED" << std::endl;
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
  LOG_info << millis() << " ms: initLeakWaterSensor(): leakWaterDetected= " << leakWaterDetected << std::endl;

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
 * @brief 	returns current in ampere from acCurrentAnalogReadAmplitudeMillivolt
 * @param 	void
 * @return 	rms current in ampere
 *********************************************************************/
double radiator::ExternalSensors::getAcCurrentAmpereRMS()
{
  auto peakCurrent = (double)(acCurrentAnalogReadAmplitudeMillivolt) * (double)AC_CURRENT_SENSOR_SCALE_AMPERE_PER_MILLIVOLT;

  return peakCurrent * 0.70710678118; // calculate rms from amplitude
  // return peakCurrent / 1.41421356;
}
