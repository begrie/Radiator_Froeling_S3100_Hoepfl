/******************************************************************************
 * Name:		main.cpp
 * @brief   Überwachungstool für Fröling P2 / Lambdatronic S 3100
 *          -> hier: für ESP32
 *          -> Output auf console, flash-filesystem oder SD-card oder/und MQTT
 *          -> rudimentärer Zugriff per WiFi Async Server
 *          -> IDE: platformio
 * Created:	01.09.2022 (ESP32-adaption)
 * Author:	Original (for Raspberry Pi): Daniel Höpfl
 *          -> https://github.com/dhoepfl/Radiator
 *          Adaption to ESP32: Bernd Griesbach
 *          -> https://github.com/begrie/Radiator_Froeling_S3100_Hoepfl/tree/ESP32
 ******************************************************************************/

/***********************
 *      INCLUDES
 ***********************/
#include "config.h"
#include "debug.h"
#include "device.h"
#include "surveillance.h"
#include "output.h"
#include "files.h"
#include "externalsensors.h"

/***********************
 *      DEFINES
 ***********************/
// see config.h

/***********************
 *      MACROS
 ***********************/
// see config.h

/***********************
 * FORWARD DECLARATIONS
 ***********************/

/***********************
 * GLOBAL DEFINITIONS
 ***********************/
radiator::NetworkHandler netHandler;
radiator::OutputHandler *ptrOutHandler;
// SemaphoreHandle_t semaphoreForSurveillance = NULL;

/*********************************************************************
 * @brief 	Setup to start everything
 * @param 	void
 * @return 	void
 *********************************************************************/
void setup()
{
  Serial.begin(115200);
  delay(1000); // give serial monitor time to connect
  Serial.println("Start setup...");

  bool outputToConsole = OUTPUT_TO_CONSOLE;
  bool outputToMQTT = OUTPUT_TO_MQTT && USE_WIFI; // is turned off without WiFi

  if (USE_WIFI)
    outputToMQTT = netHandler.init(outputToMQTT, START_WEBSERVER); // on init-failure -> turn off MQTT

  debug_level = D_DEBUG_LEVEL;                // var debug_level is globally defined in debug.cpp
  std::string dataDirectory = DATA_DIRECTORY; // directory for data from radiator -> e.g. "/data" or "" for root dir
                                              // only one level is created automatically in initFilesystem()
                                              // ->all deeper directories in path must exist before instantiation of OutputHandler

  std::string pathnameToDataDirectory =
      radiator::FilesystemHandler::initFilesystem(dataDirectory); // patnameToFiles is completed with mountpoint of the filesystem
                                                                  // (/sd/dataDirectory or /littlefs/..., /spiffs/...)
  if (pathnameToDataDirectory.empty())
  {
    std::cout << "ERROR MOUNTING FILESYSTEM !!! -> Instead: data output to console (std::cout)" << std::endl;
    outputToConsole = true;
  }
  else // filesystem successfull mounted
  {
    // FS_Filehelper::listDir("/", 255, false, FILESYSTEM_TO_USE);

#if REDIRECT_STD_ERR_TO_SYSLOG_FILE
    radiator::FilesystemHandler::initRedirectStdErrToSyslogFile();
#endif
  }

  if (!OUTPUT_TO_FILE)
    pathnameToDataDirectory = ""; // empty pathname indicates "no file output" for constuctor of OutputHandler

#if USE_EXTERNAL_SENSORS
  radiator::ExternalSensors::initExternalSensors();
#endif

  ptrOutHandler = new radiator::OutputHandler(pathnameToDataDirectory, // with empty pathnameToDataDirectory -> no output to file
                                              outputToConsole,
                                              outputToMQTT);

  // Serial.println("xxxxxxxxxx");
  // semaphoreForSurveillance = xSemaphoreCreateBinary();
  // xSemaphoreGive(semaphoreForSurveillance);
  // Serial.println("00000000000");

  LOG_info
      << "\n"
      << millis() << " ms: ***************************************** END STARTUP OF ESP32 *****************************************" << std::endl;
}

/*********************************************************************
 * @brief 	Loop function
 * @param 	void
 * @return 	void
 *********************************************************************/
void loop()
{
  std::string message;
  Ticker tickerConnectToRadiatorInfo;

  try
  {
    // Serial.println("11111111111");
    // xSemaphoreTake(semaphoreForSurveillance, portMAX_DELAY);
    // Serial.println("22222222222");

    message = std::to_string(millis()) + " ms: ##### Start connecting to Froeling P2/S3100 ##### \n";
    netHandler.publishToMQTT(message, MQTT_TOPIC_SYSLOG);
    LOG_warn << message << std::endl;
    if (REDIRECT_STD_ERR_TO_SYSLOG_FILE) // for better user info also to console
      std::cout << message;

    tickerConnectToRadiatorInfo.once_ms(
        T_TIMEOUT_BETWEEN_TRANSFERS_MS + 500,
        []()
        {
          std::string msg;
          msg = std::to_string(millis()) + " ms: ##### CONNECTED to Froeling P2/S3100 #####";
          netHandler.publishToMQTT(msg, MQTT_TOPIC_SYSLOG);
          LOG_warn << msg << std::endl;
          if (REDIRECT_STD_ERR_TO_SYSLOG_FILE) // for better user info also to console
            std::cout << msg;
        });

    radiator::Surveillance surveillance(S_SERIAL_TO_RADIATOR, T_TIMEOUT_BETWEEN_TRANSFERS_MS, *ptrOutHandler);
    surveillance.main_loop();

    // Serial.println("33333333333");

    // // Create RTOS task to have a larger stacksize (for loop: 8192)
    // TaskHandle_t HandlexTaskSurveillanceLoop;
    // BaseType_t _Result = xTaskCreatePinnedToCore(
    //     [](void *parameter)
    //     {
    //       Serial.println("444444444444444");
    //       radiator::Surveillance surveillance(S_SERIAL_TO_RADIATOR, T_TIMEOUT_BETWEEN_TRANSFERS_MS, *ptrOutHandler);
    //       surveillance.main_loop(); // the most work is done here -> ends if no or lost connection
    //       xSemaphoreGive(semaphoreForSurveillance);
    //       vTaskDelete(NULL);
    //     },                            // Task function
    //     "xTaskSurveillanceLoop",      // String with name of task
    //     16384,                        // Stack size in bytes
    //     NULL,                         // Parameter passed as input of the task
    //     uxTaskPriorityGet(NULL),      // Priority of the task: higher values -> higher priority
    //                                   // with uxTaskPriorityGet(NULL)-> same priority as current task
    //     &HandlexTaskSurveillanceLoop, // Task handle (Typ: TaskHandle_t)
    //     1);                           // Core 0 or 1 (Arduino code by default on Core 1)

    // if (_Result != pdPASS)
    //   throw std::runtime_error("xTaskExternalSensors: Error creating xTask");

    // Serial.println("5555555555555");
    // xSemaphoreTake(semaphoreForSurveillance, portMAX_DELAY); // waits until xSemaphoreGive in task
    // Serial.println("66666666666666");
    tickerConnectToRadiatorInfo.detach();

    message = std::to_string(millis()) + " ms: ##### Cannot establish connection to Froeling P2/S3100 -> retry in 10s ##### \n";
    netHandler.publishToMQTT(message, MQTT_TOPIC_SYSLOG);
    LOG_warn << message << std::endl;
    if (REDIRECT_STD_ERR_TO_SYSLOG_FILE) // for better user info also to console
      std::cout << message;

    sleep(10);
    // xSemaphoreGive(semaphoreForSurveillance);
  }
  catch (const char *&error)
  {
    message = std::to_string(millis()) + " ms: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                                         "!!!!! Failed to open serial device or filesystem -> restarting ESP32 in 10s !!!!!\n"
                                         "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
    netHandler.publishToMQTT(message, "/status");
    LOG_fatal << message << std::endl;

    sleep(10);
    esp_restart();
  }
}
