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

#include "esp32/rom/rtc.h" //to get and print the reset reason
static const char *resetReason[]{
    "NO_MEAN",
    "POWERON_RESET: Vbat power on reset",
    "NO_MEAN",
    "SW_RESET: Software reset digital core",
    "OWDT_RESET: Legacy watch dog reset digital core",
    "DEEPSLEEP_RESET: Deep Sleep reset digital core",
    "SDIO_RESET: Reset by SLC module, reset digital core",
    "TG0WDT_SYS_RESET: Timer Group0 Watch dog reset digital core",
    "TG1WDT_SYS_RESET: Timer Group1 Watch dog reset digital core",
    "RTCWDT_SYS_RESET: RTC Watch dog Reset digital core",
    "INTRUSION_RESET: Instrusion tested to reset CPU",
    "TGWDT_CPU_RESET: Time Group reset CPU",
    "SW_CPU_RESET: Software reset CPU",
    "RTCWDT_CPU_RESET: RTC Watch dog Reset CPU",
    "EXT_CPU_RESET: for APP CPU, reseted by PRO CPU",
    "RTCWDT_BROWN_OUT_RESET: Reset when the vdd voltage is not stable",
    "RTCWDT_RTC_RESET: RTC Watch dog reset digital core and rtc module",
    "NO_MEAN"};

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
void print_reset_reason(int reason);

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

  int resetReasonCPU0 = rtc_get_reset_reason(0);
  int resetReasonCPU1 = rtc_get_reset_reason(1);
  Serial.printf("resetReason CPU0: %s \n", resetReason[resetReasonCPU0]);
  Serial.printf("resetReason CPU1: %s \n", resetReason[resetReasonCPU1]);

  heap_caps_register_failed_alloc_callback(
      [](size_t requested_size, uint32_t caps, const char *function_name)
      {
        char tmp[192];
        snprintf(tmp, sizeof(tmp), "%lu ms: %s was called but failed to allocate %d bytes with 0x%X capabilities. \n", millis(), function_name, requested_size, caps);
        netHandler.publishToMQTT(tmp, "/memerror");
        Serial.println(tmp);
        LOG_fatal << tmp << std::endl;

        //  VORSICHT:russisch-> nur mit TOR öffnen https://kotyara12.ru/iot/remote_esp32_backtrace/

        // Serial.printf("%lu ms: %s was called but failed to allocate %d bytes with 0x%X capabilities. \n", millis(), function_name, requested_size, caps);
        // LOG_fatal << millis() << " ms: " << function_name << " was called but failed to allocate " << requested_size << " bytes with " << std::hex << caps << std::dec << " capabilities." << std::endl;
      });

  bool outputToConsole = OUTPUT_TO_CONSOLE;
  bool outputToMQTT = OUTPUT_TO_MQTT && USE_WIFI; // is turned off without WiFi

  try
  {
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

    std::string message = "last reset reason CPU0= " + (std::string)resetReason[resetReasonCPU0] + " / CPU1= " + (std::string)resetReason[resetReasonCPU1];
    netHandler.publishToMQTT(message, "/error");
    LOG_error << message << std::endl;

    LOG_info << "\n"
             << millis() << " ms: ***************************************** END STARTUP OF ESP32 *****************************************" << std::endl;

    LOG_WARN(LOG_warn << "test";)
    LOG_INFO(LOG_info << "test";)
  }
  catch (const char *&error)
  {
    std::string message = std::to_string(millis()) + " ms: setup: exception thrown: " + error + " / " + *&error +
                          ", Uptime : " + std::to_string(millis() / (1000 * 60)) + " min; Heap: " +
                          std::to_string(ESP.getFreeHeap()) + " / " + std::to_string(ESP.getMinFreeHeap()) + " / " + std::to_string(ESP.getMaxAllocHeap());
    LOG_fatal << message << std::endl;
    netHandler.publishToMQTT(message, "/crash");

    sleep(10);
    esp_restart();
  }
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
    message = std::to_string(millis()) + " ms: ##### Start connecting to Froeling P2/S3100 ##### \n";
    netHandler.publishToMQTT(message, MQTT_SUBTOPIC_SYSLOG);
    LOG_warn << message << std::endl;
    if (REDIRECT_STD_ERR_TO_SYSLOG_FILE) // for better user info also to console
      std::cout << message;

    tickerConnectToRadiatorInfo.once_ms(
        T_TIMEOUT_BETWEEN_TRANSFERS_MS + 500,
        []()
        {
          std::string msg;
          msg = std::to_string(millis()) + " ms: ##### CONNECTED to Froeling P2/S3100 #####";
          netHandler.publishToMQTT(msg, MQTT_SUBTOPIC_SYSLOG);
          LOG_warn << msg << std::endl;
          if (REDIRECT_STD_ERR_TO_SYSLOG_FILE) // for better user info also to console
            std::cout << msg << std::endl;
        });

    radiator::Surveillance surveillance(S_SERIAL_TO_RADIATOR, T_TIMEOUT_BETWEEN_TRANSFERS_MS, *ptrOutHandler);
    surveillance.main_loop();

    tickerConnectToRadiatorInfo.detach();

    message = std::to_string(millis()) + " ms: ##### Cannot establish connection to Froeling P2/S3100 -> retry in 10s ##### \n";
    netHandler.publishToMQTT(message, MQTT_SUBTOPIC_SYSLOG);
    LOG_warn << message << std::endl;
    if (REDIRECT_STD_ERR_TO_SYSLOG_FILE) // for better user info also to console
      std::cout << message;

    LOG_warn << millis() << " ms: main: uxTaskGetStackHighWaterMark(NULL)= " << uxTaskGetStackHighWaterMark(NULL) << std::endl;

    sleep(10);
  }
  catch (const char *&error)
  {
    message = std::to_string(millis()) + " ms: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                                         "!!!!! Failed to open serial device or filesystem -> restarting ESP32 in 10s !!!!!\n"
                                         "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
    netHandler.publishToMQTT(message, MQTT_SUBTOPIC_SYSLOG);
    LOG_fatal << message << std::endl;

    sleep(10);
    esp_restart();
  }
}

void print_reset_reason(int reason)
{
  Serial.println(resetReason[reason]);

  switch (reason)
  {
  case 1:
    Serial.println("POWERON_RESET: Vbat power on reset");
    break; /**<1,  Vbat power on reset*/
  case 3:
    Serial.println("SW_RESET: Software reset digital core");
    break; /**<3,  Software reset digital core*/
  case 4:
    Serial.println("OWDT_RESET: Legacy watch dog reset digital core");
    break; /**<4,  Legacy watch dog reset digital core*/
  case 5:
    Serial.println("DEEPSLEEP_RESET: Deep Sleep reset digital core");
    break; /**<5,  Deep Sleep reset digital core*/
  case 6:
    Serial.println("SDIO_RESET: Reset by SLC module, reset digital core");
    break; /**<6,  Reset by SLC module, reset digital core*/
  case 7:
    Serial.println("TG0WDT_SYS_RESET: Timer Group0 Watch dog reset digital core");
    break; /**<7,  Timer Group0 Watch dog reset digital core*/
  case 8:
    Serial.println("TG1WDT_SYS_RESET: Timer Group1 Watch dog reset digital core");
    break; /**<8,  Timer Group1 Watch dog reset digital core*/
  case 9:
    Serial.println("RTCWDT_SYS_RESET: RTC Watch dog Reset digital core");
    break; /**<9,  RTC Watch dog Reset digital core*/
  case 10:
    Serial.println("INTRUSION_RESET: Instrusion tested to reset CPU");
    break; /**<10, Instrusion tested to reset CPU*/
  case 11:
    Serial.println("TGWDT_CPU_RESET: Time Group reset CPU");
    break; /**<11, Time Group reset CPU*/
  case 12:
    Serial.println("SW_CPU_RESET: Software reset CPU");
    break; /**<12, Software reset CPU*/
  case 13:
    Serial.println("RTCWDT_CPU_RESET: RTC Watch dog Reset CPU");
    break; /**<13, RTC Watch dog Reset CPU*/
  case 14:
    Serial.println("EXT_CPU_RESET: for APP CPU, reseted by PRO CPU");
    break; /**<14, for APP CPU, reseted by PRO CPU*/
  case 15:
    Serial.println("RTCWDT_BROWN_OUT_RESET: Reset when the vdd voltage is not stable");
    break; /**<15, Reset when the vdd voltage is not stable*/
  case 16:
    Serial.println("RTCWDT_RTC_RESET: RTC Watch dog reset digital core and rtc module");
    break; /**<16, RTC Watch dog reset digital core and rtc module*/
  default:
    Serial.println("NO_MEAN");
  }
}
