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

  debug_level = D_DEBUG_LEVEL; // var debug_level is globally defined in debug.cpp

  std::string pathnameToDataDirectory =
      radiator::FilesystemHandler::initFilesystem(DATA_DIRECTORY); // DATA_DIRECTORY is completed with mountpoint of the filesystem
                                                                   // (/sd/dataDirectory or /littlefs/..., /spiffs/...)

  if (pathnameToDataDirectory.empty())
  {
    std::cout << getMillisAndTime() << "ERROR MOUNTING FILESYSTEM !!! -> Instead: data output to console (std::cout)" << std::endl;
    outputToConsole = true;
  }
  else // filesystem successfull mounted
  {
    // FS_Filehelper::listDir("/", 255, false, FILESYSTEM_TO_USE);

#if REDIRECT_STD_ERR_TO_SYSLOG_FILE
    if (!pathnameToDataDirectory.empty())
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

  RADIATOR_LOG_INFO("\n" << getMillisAndTime()
                         << "***************************************** END STARTUP OF ESP32 *****************************************" << std::endl;)
}

/*********************************************************************
 * @brief 	Loop function
 * @param 	void
 * @return 	void
 *********************************************************************/
void loop()
{
  std::string message;
  message.reserve(800); // an attempt to avoid heap fragmentation
  Ticker tickerConnectToRadiatorInfo;

  try
  {
    message = getMillisAndTime() + "##### Start connecting to Froeling P2/S3100 ##### \n";
    netHandler.publishToMQTT(message, MQTT_SUBTOPIC_SYSLOG);
    RADIATOR_LOG_WARN(message << std::endl;)
    if (REDIRECT_STD_ERR_TO_SYSLOG_FILE) // for better user info also to console
      std::cout << message;

    tickerConnectToRadiatorInfo.once_ms(
        T_TIMEOUT_BETWEEN_TRANSFERS_MS + 500,
        []()
        {
          char msg[80];
          snprintf(msg, sizeof(msg), "%s ##### CONNECTED to Froeling P2/S3100 #####", getMillisAndTime().c_str());
          // std::string msg;
          // msg = std::to_string(millis()) + " ms: ##### CONNECTED to Froeling P2/S3100 #####";
          netHandler.publishToMQTT(msg, MQTT_SUBTOPIC_SYSLOG);
          RADIATOR_LOG_WARN(msg << std::endl;)
          if (REDIRECT_STD_ERR_TO_SYSLOG_FILE) // for better user info also to console
            std::cout << msg << std::endl;
        });

    radiator::Surveillance surveillance(S_SERIAL_TO_RADIATOR, T_TIMEOUT_BETWEEN_TRANSFERS_MS, *ptrOutHandler);
    surveillance.main_loop();

    tickerConnectToRadiatorInfo.detach();

    message = getMillisAndTime() + "##### Cannot establish connection to Froeling P2/S3100 -> retry in 10s ##### \n";
    netHandler.publishToMQTT(message, MQTT_SUBTOPIC_SYSLOG);
    RADIATOR_LOG_WARN(message << std::endl;)
    if (REDIRECT_STD_ERR_TO_SYSLOG_FILE) // for better user info also to console
      std::cout << message;

    // DEBUG_STACK_HIGH_WATERMARK;

    sleep(10);
  }
  catch (const char *&error)
  {
    message = getMillisAndTime() +
              " ms: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
              "!!!!! *&error= " +
              *&error + ", error= " + error +
              "!!!!! Failed to open serial device or filesystem -> restarting ESP32 in 10s !!!!!\n"
              "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
    netHandler.publishToMQTT(message, MQTT_SUBTOPIC_SYSLOG);
    LOG_fatal << message << std::endl;

    sleep(10);
    esp_restart();
  }
}
