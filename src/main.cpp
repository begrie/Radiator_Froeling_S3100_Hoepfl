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
#include <Arduino.h>
#include <LittleFS.h>
#include <SPIFFS.h>
#include "FS_Filehelper.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include "debug.h"
#include "device.h"
#include "surveillance.h"
#include "output.h"

/*********************
 *      DEFINES
 *********************/
#define D_DEBUG_LEVEL 3                       // 0 - No logging
                                              // 1 - Errors
                                              // 2 - Warnings
                                              // 3 - Infos
                                              // 4 - Debug
                                              // 5 - Trace
#define REDIRECT_STD_ERR_TO_SYSLOG_FILE false // debug output to file instead to console/Serial
#define SYSLOG_FILENAME "/syslog1.log"        // name without path -> is stored on LittleFS
                                              // -> watch the needed filespace:
                                              //    makes only sense at long term operation with D_DEBUG_LEVEL 1 - Errors

#define S_SERIAL_TO_RADIATOR "Serial2" // Serial device for connection to radiator device P2/S3100
// #define SERIAL_TO_RADIATOR_RX 16 // ESP32-standard: 16 -> needs to be changed in serial.cpp if neccessary
// #define SERIAL_TO_RADIATOR_TX 17 // ESP32-standard: 17 -> needs to be changed in serial.cpp if neccessary
#define T_TIMEOUT_BETWEEN_TRANSFERS_MS 5000 // Maximum time allowed between transfers from P2/S3100 radiator device
                                            // until connection is considered stale.

// Where to write the received values to:
#define OUTPUT_TO_CONSOLE false
#define OUTPUT_TO_MQTT false
#define OUTPUT_TO_FILE true
#define FILESYSTEM_TO_USE LittleFS           // LittleFS or SD (or SPIFFS)
#define FILESYSTEM_BASE_PATH "/littlefs"     // "/littlefs" or "/sd" (or "/spiffs") -> must correspond to above filesystem definition
#define DATA_DIRECTORY "/P2_Logging"         // directory for radiator data -> e.g. "/logging" or "" for root dir of used filesystem
#define MIN_FREE_KILOBYTES_ON_FILESYSTEM 300 // if limit is undercut -> oldest logfiles are deleted

#define USE_WIFI true
#define WIFI_HOSTNAME "Heizung_Froeling_P2"
#define USE_MQTT false
#define MQTT_BROKER

// !!! SEE MORE #defines in output.h. FOR CONTROLLING THE OUTPUT BEHAVIOUR !!!

/*********************
 *      MACROS
 *********************/
#define XSTRINGIFY(a) STRINGIFY(a)
#define STRINGIFY(a) #a

/********************************************
 * FORWARD DECLARATIONS
 ********************************************/
void xTaskWatchdogAndMaintenance(void *parameter);
std::string initFilesystem(std::string dataDirectory);
void checkFreeSpaceOnFilesystem();
void writeSyslogfile();
bool configureWiFiAndMQTT();
void configureWebserver();

/********************************************
 * GLOBAL DEFINITIONS
 ********************************************/
std::string pathnameToDataDirectory; // with mountpoint/mountdir of filesystem
bool outputToConsole = OUTPUT_TO_CONSOLE;
bool outputToMQTT = OUTPUT_TO_MQTT;

//#if REDIRECT_STD_ERR_TO_SYSLOG_FILE
std::ofstream syslogFileStream;
std::string syslogPathName = FILESYSTEM_BASE_PATH SYSLOG_FILENAME;
//#endif

/*********************************************************************
 * @brief 	Setup to start everything
 * @param 	void
 * @return 	void
 *********************************************************************/
void setup()
{
  Serial.begin(115200);
  delay(1000); // give serial monitor time to connect

  debug_level = D_DEBUG_LEVEL;
  std::string dataDirectory = DATA_DIRECTORY; // directory for data from radiator -> e.g. "/data" or "" for root dir
                                              // all directories in path must exist before instantiation of OutputHandler
                                              // (only one level is created automatically in initFilesystem())

  configureWiFiAndMQTT();

  configureWebserver();

  pathnameToDataDirectory = initFilesystem(dataDirectory); // LittleFS (or SPIFFS or SD), directory for data
                                                           // patnameToFiles is completed with mountpoint of the filesystem
  if (pathnameToDataDirectory.empty())
  {
    std::cout << "ERROR MOUNTING FILESYSTEM !!! -> Instead: data output to console (std::cout)" << std::endl;
    outputToConsole = true;
  }
  else // filesystem successfull mounted
  {
#if REDIRECT_STD_ERR_TO_SYSLOG_FILE
    // Ausgaben auf Logfile umleiten -> geht so nur mit regelmäßigem close und re-open des streams -> BUG bei LittleFS??
    LOG_info << millis()
             << " ms: !!! ALL DEBUG OUTPUT TO std::cerr IS NOW REDIRECTED TO FILE " << syslogPathName << " !!!\n"
             << "  \t !!! -> NO OUTPUT TO CONSOLE / SERIAL ANY MORE !!!" << std::endl;
    syslogFileStream.open(syslogPathName, std::ofstream::out | std::ofstream::app);
    if (syslogFileStream && syslogFileStream.is_open() && syslogFileStream.good())
    {
      //  syslogFilestream.rdbuf()->pubsetbuf(0, 0);  //sollte ohne buffer direkt schreiben -> kein Wirkung bei littlefs
      // syslogFileStream << "Syslogfile " << syslogPathName << " opened successfull " << std::endl;
      std::cerr.rdbuf(syslogFileStream.rdbuf()); // redirect by assigning buffer of file stream
                                                 // std::cerr.tie(&syslogFilestream);
      // syslogFileStream << "Redirect output from std::err to syslogfile " << std::endl;
    }
    else
    {
      LOG_info << millis() << " ms: ERROR opening syslog file " << syslogPathName << std::endl;
    }
#endif

    TaskHandle_t HandlexTaskWatchdogAndMaintenance;

    // Create RTOS task
    BaseType_t _Result = xTaskCreatePinnedToCore(
        xTaskWatchdogAndMaintenance,        // Task function
        "xTaskWatchdogAndMaintenance",      // String with name of task
        4096,                               // Stack size in bytes
        NULL,                               // Parameter passed as input of the task
        uxTaskPriorityGet(NULL),            // Priority of the task: higher values -> higher priority
                                            // with uxTaskPriorityGet(NULL)-> same priority as current task
        &HandlexTaskWatchdogAndMaintenance, // Task handle (Typ: TaskHandle_t)
        1);                                 // Core 0 or 1 (Arduino code by default on Core 1)

    if (_Result != pdPASS)
    {
      throw std::runtime_error("Start_Watchdog_and_Maintenance_Task: Error creating xTask");
    }
  }

  if (!OUTPUT_TO_FILE)
    pathnameToDataDirectory = ""; // empty pathname indicates "no file output" for constuctor of OutputHandler

  LOG_info << "\n"
           << millis() << " ms: ***************************************** STARTUP OF ESP32 *****************************************" << std::endl;
}

/*********************************************************************
 * @brief 	Loop function
 * @param 	void
 * @return 	void
 *********************************************************************/
void loop()
{
  try
  {
    LOG_info << millis() << " ms: ###################### Starting main loop ######################" << std::endl;

    radiator::OutputHandler outHandler(pathnameToDataDirectory, // with empty pathnameToDataDirectory -> no output to file
                                       outputToConsole,
                                       outputToMQTT);
    radiator::Surveillance surveillance(S_SERIAL_TO_RADIATOR, T_TIMEOUT_BETWEEN_TRANSFERS_MS, outHandler);
    surveillance.main_loop();

    LOG_info << millis() << " ms: ###################### Main loop ended, restarting in 10s ######################" << std::endl;
    sleep(10);
  }
  catch (const char *&error)
  {
    LOG_fatal << millis() << " ms: !!!!!!!!!!!!!! Failed to open serial device or filesystem -> restarting ESP32 in 10s !!!!!!!!!!!!!!\n"
              << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    sleep(10);
    esp_restart();
  }
}

/*********************************************************************************************
 *@brief   task for cyclic actions like filesystem space check
 *         and watchdog regarding possible malfunctions -> still TODO
 *@param   pointer to parameter for the task -> nothing here!
 *@return  void
 *********************************************************************************************/
void xTaskWatchdogAndMaintenance(void *parameter)
{
  vTaskDelay(pdMS_TO_TICKS(10000)); // wait 10 sec. at startup

  const auto intervallForFilesystemCheckSec = 6 * 60 * 60; // execute all 6 hours
  ulong nextFilesystemCheck = 0;

#if REDIRECT_STD_ERR_TO_SYSLOG_FILE
  const auto intervallForWriteSyslogfileSec = 15; //*60; // all 15 minutes
  // const auto intervallForWriteSyslogfileSec = 15 * 60; // all 15 minutes
  ulong nextWriteSyslogfile = 0;
#else
  const auto intervallForWriteSyslogfileSec = INT_MAX;
#endif

  const auto watchDogIntervallSec = std::min(intervallForFilesystemCheckSec, intervallForWriteSyslogfileSec);

  while (true) // the xTask runs "endless"
  {
    if (millis() > nextFilesystemCheck)
    {
      checkFreeSpaceOnFilesystem();
      nextFilesystemCheck = millis() + intervallForFilesystemCheckSec * 1000;
    }

#if REDIRECT_STD_ERR_TO_SYSLOG_FILE
    if (millis() > nextWriteSyslogfile)
    {
      writeSyslogfile();
      nextWriteSyslogfile = millis() + intervallForWriteSyslogfileSec * 1000;
    }
#endif

    vTaskDelay(watchDogIntervallSec * 1000 / portTICK_PERIOD_MS); // execute all xx seconds
  }
  assert(false); //-> we must never get here -> if we do -> restart
}

/*********************************************************************
 * @brief 	Initialization of filesystem for use with ostream
 * @param 	Filesystem to use: LittleFS (or SPIFFS) or SD
 * @param   Directory for data received from radiator e.g. "/data" or "" for root dir
 * @return 	Pathname to data directory with mountpoint/mountdir like "/littlefs/data"
 *********************************************************************/
std::string initFilesystem(std::string dataDirectory)
{
  std::string basepath;
  std::string fsName;

  if ((void *)&FILESYSTEM_TO_USE == (void *)&LittleFS)
  {
    fsName = "LittleFS";
    // basepath = "/littlefs"; // standard mountpoint(mountpath)
  }
  else if ((void *)&FILESYSTEM_TO_USE == (void *)&SD)
  {
    fsName = "SD";
    // basepath = "/sd"; // standard mountpoint(mountpath)
  }
  else if ((void *)&FILESYSTEM_TO_USE == (void *)&SPIFFS)
  {
    fsName = "SPIFFS";
    // basepath = "/spiffs"; // standard mountpoint(mountpath)
  }
  basepath = FILESYSTEM_BASE_PATH; // standard mountpoint(mountpath)

  LOG_info << "\nSelected filesystem for output to file: " << fsName << std::endl;

  if (!FILESYSTEM_TO_USE.begin(false))
  {
    LOG_info << fsName << " mount failed -> Try to format file system for " << fsName << " -> please wait ..." << std::endl;
    if (!FILESYSTEM_TO_USE.begin(true)) // open with format filesystem
    {
      LOG_error << fsName << " formatting not possible -> NO LOCAL DATA STORAGE AVAILABLE" << std::endl;
      return "";
    }
    LOG_info << "Formatting of filesystem for " << fsName << " successfull" << std::endl;
  }

  // FS_Filehelper::deleteDirectory("/");
  // FILESYSTEM_TO_USE.format();

  LOG_info << "Filesystem   " << fsName << "   was successfull mounted at basepath:  " << basepath
           << "  -> Available space " << ((FILESYSTEM_TO_USE.totalBytes() - FILESYSTEM_TO_USE.usedBytes()) / 1024)
           << " kB from total " << (FILESYSTEM_TO_USE.totalBytes() / 1024) << " kB" << std::endl;

  if (!dataDirectory.empty())
  {
    if (dataDirectory.front() != '/')
      dataDirectory = '/' + dataDirectory;

    if (FILESYSTEM_TO_USE.exists((dataDirectory).c_str())) // it seems that LittleFS creates a new dir on its own if it did not exist
    {
      LOG_info << "Directory " << dataDirectory << " already exists" << std::endl;
    }
    else
    {
      LOG_info << "dataDirectory " << dataDirectory << " did not exist -> try to create it ..." << std::endl;
      if (FILESYSTEM_TO_USE.mkdir(dataDirectory.c_str()))
      {
        LOG_info << "... " << dataDirectory << " was successfully created" << std::endl;
      }
      else
      {
        LOG_error << "... " << dataDirectory << " CAN NOT BE CREATED" << std::endl;
        dataDirectory = "";
      }
    }
  }

  basepath += dataDirectory;
  LOG_info << "Whole output path is  " << basepath << std::endl;

  // FS_Filehelper::writeFile(SYSLOG_FILENAME, "Log1 \nLog2 \nLog3 \n");
  // FS_Filehelper::writeFile(((std::string)DATA_DIRECTORY + "/2022-09-13").c_str(), "11111111111111111\n22222222222222222\n33333333333333\n");
  // FS_Filehelper::writeFile(((std::string)DATA_DIRECTORY + "/2022-09-12").c_str(), "77777777\n8888888888\n999999999999\n");
  // FS_Filehelper::writeFile(
  //     ((std::string)DATA_DIRECTORY + "/2022-09-13.log").c_str(),
  //     "Date; Time;  [S];  [S]; Zustand [N]; ROST [N]; Kesseltemp [N]; Abgastemp. [N]; Abgas. SW  [N]; KessStellGr [N]; Saugzug   [N]; Zuluftgebl [N]; Einschub [N]; Fuellst.: [N]; Feuerraumt [N]; Puffert.ob [N]; Puffert.un [N]; Puffer Pu. [N]; Außentemp [N]; Vorlauft.1sw [N]; Vorlauft.1 [N]; Vorlauft.2sw [N]; Vorlauft.2 [N]; KTY6_H2 [N]; KTY7_H2 [N]; Brenn.ST [N]; Laufzeit: [N]; Boardtemp. [N]; Die Kesseltemp. soll sein [N];\n"
  //     "2022-09-13; 10:52:00; Ausgeschaltet ;   Brenner Aus  ; 1; 0; 18°; 23°; 33°; 51%; 0%; 0%; 0%; 99.9%; 394°; 18°; 18°; 0%; 18°; 0°; 20°; 0°; 18°; 127°; 127°; 10461; 15878h; 23°; 75°;\n"
  //     "2022-09-13; 10:52:11; Ausgeschaltet ;   Brenner Aus  ; 1; 0; 18°; 23°; 33°; 51%; 0%; 0%; 0%; 99.9%; 394°; 18°; 18°; 0%; 18°; 0°; 20°; 0°; 18°; 127°; 127°; 10461; 15878h; 23°; 75°;\n"
  //     "2022-09-13; 10:52:22; Ausgeschaltet ;   Brenner Aus  ; 1; 0; 18°; 23°; 33°; 51%; 0%; 0%; 0%; 99.9%; 394°; 18°; 18°; 0%; 18°; 0°; 20°; 0°; 18°; 127°; 127°; 10461; 15878h; 23°; 75°;\n");
  // FS_Filehelper::writeFile(
  //     ((std::string)DATA_DIRECTORY + "/2022-09-12.log").c_str(),
  //     "Date; Time;  [S];  [S]; Zustand [N]; ROST [N]; Kesseltemp [N]; Abgastemp. [N]; Abgas. SW  [N]; KessStellGr [N]; Saugzug   [N]; Zuluftgebl [N]; Einschub [N]; Fuellst.: [N]; Feuerraumt [N]; Puffert.ob [N]; Puffert.un [N]; Puffer Pu. [N]; Außentemp [N]; Vorlauft.1sw [N]; Vorlauft.1 [N]; Vorlauft.2sw [N]; Vorlauft.2 [N]; KTY6_H2 [N]; KTY7_H2 [N]; Brenn.ST [N]; Laufzeit: [N]; Boardtemp. [N]; Die Kesseltemp. soll sein [N];\n"
  //     "2022-09-12; 10:52:00; Ausgeschaltet ;   Brenner Aus  ; 1; 0; 18°; 23°; 33°; 51%; 0%; 0%; 0%; 99.9%; 394°; 18°; 18°; 0%; 18°; 0°; 20°; 0°; 18°; 127°; 127°; 10461; 15878h; 23°; 75°;\n"
  //     "2022-09-12; 10:52:11; Ausgeschaltet ;   Brenner Aus  ; 1; 0; 18°; 23°; 33°; 51%; 0%; 0%; 0%; 99.9%; 394°; 18°; 18°; 0%; 18°; 0°; 20°; 0°; 18°; 127°; 127°; 10461; 15878h; 23°; 75°;\n"
  //     "2022-09-12; 10:52:22; Ausgeschaltet ;   Brenner Aus  ; 1; 0; 18°; 23°; 33°; 51%; 0%; 0%; 0%; 99.9%; 394°; 18°; 18°; 0%; 18°; 0°; 20°; 0°; 18°; 127°; 127°; 10461; 15878h; 23°; 75°;\n");
  // FS_Filehelper::writeFile(
  //     ((std::string)DATA_DIRECTORY + "/2022-08-22.log").c_str(),
  //     "Date; Time;  [S];  [S]; Zustand [N]; ROST [N]; Kesseltemp [N]; Abgastemp. [N]; Abgas. SW  [N]; KessStellGr [N]; Saugzug   [N]; Zuluftgebl [N]; Einschub [N]; Fuellst.: [N]; Feuerraumt [N]; Puffert.ob [N]; Puffert.un [N]; Puffer Pu. [N]; Außentemp [N]; Vorlauft.1sw [N]; Vorlauft.1 [N]; Vorlauft.2sw [N]; Vorlauft.2 [N]; KTY6_H2 [N]; KTY7_H2 [N]; Brenn.ST [N]; Laufzeit: [N]; Boardtemp. [N]; Die Kesseltemp. soll sein [N];\n"
  //     "2022-08-22; 10:52:00; Ausgeschaltet ;   Brenner Aus  ; 1; 0; 18°; 23°; 33°; 51%; 0%; 0%; 0%; 99.9%; 394°; 18°; 18°; 0%; 18°; 0°; 20°; 0°; 18°; 127°; 127°; 10461; 15878h; 23°; 75°;\n"
  //     "2022-08-22; 10:52:11; Ausgeschaltet ;   Brenner Aus  ; 1; 0; 18°; 23°; 33°; 51%; 0%; 0%; 0%; 99.9%; 394°; 18°; 18°; 0%; 18°; 0°; 20°; 0°; 18°; 127°; 127°; 10461; 15878h; 23°; 75°;\n"
  //     "2022-08-22; 10:52:22; Ausgeschaltet ;   Brenner Aus  ; 1; 0; 18°; 23°; 33°; 51%; 0%; 0%; 0%; 99.9%; 394°; 18°; 18°; 0%; 18°; 0°; 20°; 0°; 18°; 127°; 127°; 10461; 15878h; 23°; 75°;\n");

  FS_Filehelper::listDir("/", 255, false, FILESYSTEM_TO_USE);

  return basepath;
}

/*********************************************************************
 * @brief 	check free space on filesystem and remove oldest files
 * @param 	void
 * @return 	void
 *********************************************************************/
void checkFreeSpaceOnFilesystem()
{
  auto freekBytes = (FILESYSTEM_TO_USE.totalBytes() - FILESYSTEM_TO_USE.usedBytes()) / 1024;

  LOG_info
      << millis() << " ms: Checking free space limit (" << MIN_FREE_KILOBYTES_ON_FILESYSTEM
      << " kB) for the logging data on the filesystem:\n"
      << " \t Available " << freekBytes
      << " kB from total " << (FILESYSTEM_TO_USE.totalBytes() / 1024) << " kB \n";

  if (freekBytes > MIN_FREE_KILOBYTES_ON_FILESYSTEM)
  {
    LOG_info << " \t -> OK: Enough free space" << std::endl;
  }
  else
  {
    LOG_info << " \t -> Min. free space limit was undercut -> Oldest logfile in directory "
             << DATA_DIRECTORY << " will be deleted ..." << std::endl;

    auto rootDir = FILESYSTEM_TO_USE.open(DATA_DIRECTORY, FILE_READ);
    if (rootDir)
    {
      std::list<std::string> filesList;
      File file = rootDir.openNextFile();
      while (file)
      {
        // LOG_info << "file " << file.path() << std::endl;
        if (!file.isDirectory())
          filesList.push_back(file.path());

        file = rootDir.openNextFile();
      }

      filesList.sort();
      LOG_info << "Sorted files list: \n";
      for (auto el : filesList)
        LOG_info << el << "\n";

      if (FILESYSTEM_TO_USE.remove(filesList.front().c_str()))
        LOG_info << "File   " << filesList.front() << " deleted" << std::endl;
      else
        LOG_error << millis() << " ms: ERROR: xTask_Watchdog_and_Maintenance at Check for free space on filesystem -> FILE  "
                  << filesList.front()
                  << "   CANNOT BE DELETED -> NO SPACE FREED ON FILESYSTEM (" << freekBytes << " kB available)" << std::endl;
    }
  }
}

/*********************************************************************
 * @brief 	in case the output to std::err is redirected to a syslog file
 *          -> the filebuffer is only written if file is closed (BUG?)
 *          -> for that we close and reopen the file here
 * @param 	void
 * @return 	void
 *********************************************************************/
void writeSyslogfile()
{
#if REDIRECT_STD_ERR_TO_SYSLOG_FILE
  if (syslogFileStream && syslogFileStream.is_open())
  {
    syslogFileStream.close();
    syslogFileStream.open(syslogPathName, std::ofstream::out | std::ofstream::app);
  }
#endif
}

/*********************************************************************
 * @brief 	Configure connection, diconnection, SSID and password settings for WiFi
 *          -> WiFi connection and reconnection attempts itself and made
 *             in a parallel background task from the WiFi class
 *          -> Change WiFi or MQTT settings from console/Serial by pressing the big yellow button
 *             (QUIT_BUZZER_BUTTON_PIN) at startup of the ESP32
 * @param 	void
 * @return 	true  -> settings are OK (->set persistent) OR WiFi is switched off as requested by user input
 *          false -> no persitent settings are available
 *********************************************************************/
bool configureWiFiAndMQTT()
{
  // install WiFi callbacks
  WiFi.onEvent(
      [](WiFiEvent_t _WiFi_Event, WiFiEventInfo_t _WiFi_Event_Info)
      {
        LOG_info << "\nWiFi connected to... \n"
                 << "\t WLAN / SSID: \t" << WiFi.SSID().c_str() << "\n"
                 << "\t Signal: \tRSSI " << std::to_string(WiFi.RSSI()) << "\n"
                 << "\t Hostname: \t" << WiFi.getHostname() << "\n"
                 << "\t IP address: \t" << WiFi.localIP().toString().c_str() << "\n"
                 << "\t Gateway IP: \t" << WiFi.gatewayIP().toString().c_str() << "\n"
                 << "\t Network IP: \t" << WiFi.networkID().toString().c_str() << "\n"
                 << std::endl;

        pinMode(BUZZER_PIN, OUTPUT);
        bool OnOff = 1;
        for (int i = 0; i < 5; i++)
        {
          digitalWrite(BUZZER_PIN, OnOff);
          OnOff = !OnOff;
          vTaskDelay(pdMS_TO_TICKS(100));
        }
        digitalWrite(BUZZER_PIN, LOW);
      },
      WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);

  WiFi.onEvent(
      [](WiFiEvent_t _WiFi_Event, WiFiEventInfo_t _WiFi_Event_Info)
      {
        static ulong timeConnectionLostMs = 0;
        if (WiFi.status() == WL_CONNECTION_LOST)
        {
          LOG_info
              << "\nWiFi was DISCONNECTED from WLAN. System is trying to reconnect in a background task ... \n"
              << "(Info: Change WiFi or MQTT settings by pressing the big yellow button at startup of the ESP32)\n"
              << std::endl;
          timeConnectionLostMs = millis();
        }

        static ulong nextInfoOutputMs = 0;
        static const int infoOutputIntervallSec = 10 * 60; // info output all 10 minutes -> consider needed space when output is redirected to syslog file

        if (WiFi.status() == WL_NO_SSID_AVAIL && millis() >= nextInfoOutputMs)
        {
          LOG_info
              << "\nWiFi is NOT CONNECTED to WLAN -> SSID not available. The system keeps trying to reconnect in a background task ... \n"
              << "\t(Connection was lost " << ((millis() - timeConnectionLostMs) / (1000 * 60)) << " minutes ago.)\n"
              << "\t(Info: Change WiFi or MQTT settings by pressing the big yellow button at startup of the ESP32)\n"
              << std::endl;
          nextInfoOutputMs = millis() + infoOutputIntervallSec * 1000;

          pinMode(BUZZER_PIN, OUTPUT);
          bool OnOff = 1;
          for (int i = 0; i < 3; i++)
          {
            digitalWrite(BUZZER_PIN, OnOff);
            OnOff = !OnOff;
            vTaskDelay(pdMS_TO_TICKS(750));
          }
          digitalWrite(BUZZER_PIN, LOW);
        }
      },
      WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  WiFi.onEvent(
      [](WiFiEvent_t _WiFi_Event, WiFiEventInfo_t _WiFi_Event_Info)
      {
        LOG_info << "\nWiFi:\tGot IP address:\t" << WiFi.localIP().toString().c_str() << "\n"
                 << "\tGateway IP: \t" << WiFi.gatewayIP().toString().c_str() << "\n"
                 << "\tNetwork IP: \t" << WiFi.networkID().toString().c_str() << "\n"
                 << std::endl;
      },
      WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);

  // check if button on QUIT_BUZZER_BUTTON_PIN is pressed to start WiFiConfiguration
  const auto quitButtonPin = QUIT_BUZZER_BUTTON_PIN;
  pinMode(quitButtonPin, INPUT_PULLUP);

  if (digitalRead(quitButtonPin)) // HIGH -> button is NOT pressed
  {
    std::cout << "\n\nWiFi: Using persistent saved WiFi settings... \n"
              << "\t(Info: Change WiFi or MQTT settings by pressing the big yellow button at startup of the ESP32) \n"
              << std::endl;

    auto result = WiFi.begin();

    if (result == WL_CONNECT_FAILED)
    {
      std::cout << "WiFi connection failed due to missing persistent configuration" << std::endl;
      return false;
    }
  }
  else // button pressed for new configuration
  {
    std::cout << "\n\nWiFi: Button pressed at startup for WiFi and MQTT configuration:\n"
              << "\tNow starting scan for WiFi networks ..."
              << std::endl;

    auto countOfFoundNetworks = WiFi.scanNetworks(false); // scan in blocking mode

    std::cout << "Scan done -> found " << countOfFoundNetworks << " WiFi networks:" << std::endl;

    for (int i = 0; i < countOfFoundNetworks; i++)
    {
      std::cout
          << "[" << i + 1 << "] " << WiFi.SSID(i).c_str()
          << "\t RSSI=" << WiFi.RSSI(i)
          << std::endl;
    }

    auto oldTimeout = Serial.getTimeout();
    Serial.setTimeout(ULONG_MAX);

    int intInput;
    std::string strInput;
    bool inputOk = false;

    do
    {
      std::cout << "Select network by [number] or 0 to turn WiFi off:    " << std::endl;
      intInput = Serial.parseInt();
      Serial.readStringUntil('\n').c_str(); // remove line end
      intInput--;

      if (intInput < -1 || intInput > countOfFoundNetworks)
      {
        std::cout << "Input NOT VALID! Please do it again." << std::endl;
      }
      else
      {
        inputOk = true;
      }
    } while (!inputOk);

    if (intInput == -1)
    {
      std::cout << "WiFi is turned OFF" << std::endl;
      WiFi.persistent(false);
      WiFi.disconnect(true, true);   // turn off WiFi, erase AP from persistent
      Serial.setTimeout(oldTimeout); // restore previous value
      return false;
    }

    std::cout << "Please input password for WLAN with SSID  " << WiFi.SSID(intInput).c_str() << ":" << std::endl;
    strInput = Serial.readStringUntil('\r').c_str();

    Serial.setTimeout(oldTimeout); // restore previous value

    std::cout << "SSID and password are stored persistent for automatic WiFi connection at startup" << std::endl;
    WiFi.persistent(true);
    WiFi.begin(WiFi.SSID(intInput).c_str(), strInput.c_str());
  }

  WiFi.setHostname(WIFI_HOSTNAME);

  std::cout << "WiFi: Try to connect now to WLAN in background task ..." << std::endl;

  return true;
}

/*********************************************************************
 * @brief 	assemble and format system info to a std::string
 * @param   void
 * @return 	systeminfo
 *********************************************************************/
std::string get_System_Info()
{
  std::stringstream systemInfo;

  systemInfo << "Microcontroller ESP32: \n";
  systemInfo << "ESP32 chip model  : " << ESP.getChipModel() << "\n";
  systemInfo << "Chip Revision     : " << ((int)ESP.getChipRevision()) << "\n";
  systemInfo << "CPU Frequency     : " << ESP.getCpuFreqMHz() << " MHz \n";
  systemInfo << "Chip cores        : " << ((int)ESP.getChipCores()) << "\n";
  systemInfo << "Flash chip size   : " << (ESP.getFlashChipSize() / 1024) << " kBytes \n";
  systemInfo << "Flash chip speed  : " << (ESP.getFlashChipSpeed() / 1000000) << " MHz \n";
  systemInfo << "Sketch size       : " << (ESP.getSketchSize() / 1024) << " kBytes \n";
  systemInfo << "Free sketch space : " << (ESP.getFreeSketchSpace() / 1024) << " kBytes \n";
  systemInfo << "SDK version       : " << ESP.getSdkVersion() << "\n";

  systemInfo << "\n";

  systemInfo << "Uptime            : " << (millis() / (1000 * 60)) << " min \n";
  systemInfo << "Heapsize          : " << (ESP.getHeapSize() / 1024) << " kBytes \n";
  systemInfo << "Free heap         : " << (ESP.getFreeHeap() / 1024) << " kBytes \n";
  systemInfo << "Min. free heap    : " << (ESP.getMinFreeHeap() / 1024) << " kBytes \n";
  systemInfo << "Max. alloc heap   : " << (ESP.getMaxAllocHeap() / 1024) << " kBytes \n";
  systemInfo << "Free PSRAM        : " << ESP.getFreePsram() << " Bytes \n";
  systemInfo << "Filesystem        : " << XSTRINGIFY(FILESYSTEM_TO_USE) << " \n";
  systemInfo << "Basepath(Mountpt.): " << FILESYSTEM_BASE_PATH << " \n";
  systemInfo << "Used Filesystem   : " << (FILESYSTEM_TO_USE.usedBytes() / 1024) << " kBytes \n";
  systemInfo << "Free Filesystem   : " << ((FILESYSTEM_TO_USE.totalBytes() - SPIFFS.usedBytes()) / 1024) << " kBytes \n";

  systemInfo << "\n";

  systemInfo << "WiFi IP            : " << WiFi.localIP().toString().c_str() << "\n";
  systemInfo << "WiFi hostname      : " << WiFi.getHostname() << "\n";
  systemInfo << "WiFi SSID          : " << WiFi.SSID().c_str() << "\n";
  systemInfo << "WiFi status        : " << WiFi.status() << "\n";
  systemInfo << "WiFi strength RSSI : " << ((int)WiFi.RSSI()) << " dB\n";
  systemInfo << "WiFi MAC           : " << WiFi.macAddress().c_str() << "\n";
  systemInfo << "WiFi subnet        : " << WiFi.subnetMask().toString().c_str() << "\n";
  systemInfo << "WiFi gateway       : " << WiFi.gatewayIP().toString().c_str() << "\n";
  systemInfo << "WiFi DNS 1         : " << WiFi.dnsIP(0).toString().c_str() << "\n";
  systemInfo << "WiFi DNS 2         : " << WiFi.dnsIP(1).toString().c_str() << "\n";

  systemInfo << std::endl;
  std::cout << systemInfo.str();
  return systemInfo.str();
}

/*********************************************************************
 * @brief 	list and handle radiator logfiles for webserver
 * @param 	pathname for logfiles directory
 * @param 	used filesystem
 * @return 	text/html page with list of files in directory
 *          and download buttons (with javascript)
 *********************************************************************/
std::string handleLogfilesForWebserver(std::string dirname, fs::FS &fs = FILESYSTEM_TO_USE)
{
  File root = fs.open(dirname.c_str());
  if (!root || !root.isDirectory())
  {
    LOG_error << "Failed to open directory " << dirname << std::endl;
    return "";
  }

  std::stringstream filesDownloadList;
  std::stringstream filesUrlsList;

  File file = root.openNextFile();
  while (file)
  {
    if (!file.isDirectory())
    {
      filesUrlsList << "<li>"
                    << "<a href='" << file.name() << "'>"
                    << file.name() << " (" << file.size() << " bytes)   </a>"
                    << "<button type=button><a href='" << file.name() << "' download> download</a></button>"
                    << "</li>";
      filesDownloadList << "{ download: '" << file.name() << "' }";
    }
    file = root.openNextFile();
    if (file)
      filesDownloadList << ",";
  }

  std::stringstream logfilesPage;
  logfilesPage << "<!DOCTYPE HTML><html>"
               << " <head>"
               << "  <title>Pelletsheizung Froeling P2/S3100</title>"
               << "  <meta charset='UTF-8'>"
               << "  <meta name='viewport' content='width=device-width, initial-scale=1.0'>"
               << "  <link rel='icon' href='data:,'>"
               << "</head>"
               << "<script> "
               << "  function downloadAllFiles(files)"
               << "  {"
               << "      files.forEach(function(el)"
               << "      {"
               << "        let a = document.createElement('a');"
               << "        a.href = el.download;"
               << "        a.target = '_parent';"
               << "        a.download = el.filename;"
               << "        (document.body || document.documentElement).appendChild(a);"
               << "        a.click();"
               << "        a.parentNode.removeChild(a);"
               << "      });"
               << "  }"
               << "</script>"
               << "<body>"
               << "  <h2>ESP Web Server für Pelletsheizung Fröling P2/S3100</h2>"
               << "  <p>"
               << "  <button type=button onclick=\"downloadAllFiles([" << filesDownloadList.str() << "])\">Download ALL files</button>"
               << "  </p><p>"
               << "  <button type=button><a href=downloadAllAsOneFile>Create ONE FILE FROM LAST YEAR DATA for download </br>!!! THIS WILL LAST A WHILE !!!</a></button>"
               << "  </p>"
               << "  <ul>"
               << filesUrlsList.str()
               << "  </ul>"
               << "</body></html>"
               << std::endl;

  return logfilesPage.str();
}

/*********************************************************************
 * @brief 	download ALL logfiles as ONE FILE
 * @param 	void
 * @return 	void
 *********************************************************************/
std::string downloadAllAsOneFile(std::string dirname, fs::FS &fs = FILESYSTEM_TO_USE)
{
  File root = fs.open(dirname.c_str());
  if (!root || !root.isDirectory())
  {
    auto buf = "Failed to open directory " + dirname;
    LOG_error << buf << std::endl;
    return buf;
  }

  // create list from filenames for sorting and selecting timespan
  std::list<std::string> filesList;
  File file = root.openNextFile();
  while (file)
  {
    if (!file.isDirectory())
      filesList.push_back(file.path());

    file = root.openNextFile();
  }
  filesList.sort();

  // save only one year = 365 days = no. of files
  while (filesList.size() > 365)
  {
    filesList.pop_front();
  }

  // LOG_info << "Sorted files list: \n";
  // for (auto el : filesList)
  //   LOG_info << el << "\n";

  std::string subdirName = "/union";
  fs.mkdir((dirname + subdirName).c_str());
  std::string targetFilename = "/lastYear.log";
  std::string targetPathname = dirname + subdirName + targetFilename;
  fs.remove(targetPathname.c_str()); // we need an empty file
  File targetFile = fs.open(targetPathname.c_str(), FILE_APPEND);
  if (!targetFile)
  {
    auto buf = "Failed to open target file  " + targetPathname + "  for writing";
    LOG_error << buf << std::endl;
    return buf;
  }

  for (auto el : filesList)
  {
    fs::File sourceFile = fs.open(el.c_str(), FILE_READ);
    if (!sourceFile.isDirectory() && strcmp(sourceFile.path(), targetFile.path()) != 0)
    {
      size_t written_bytes = 0;
      while (sourceFile.available())
      {
        written_bytes += targetFile.write(sourceFile.read());
        // std::cout << '.';
      }
      LOG_info << "  Written " << written_bytes << " bytes from " << sourceFile.name() << " to " << targetFile.name() << std::endl;
    }
    sourceFile.close();
  }
  targetFile.flush();

  std::stringstream page;
  page << "<!DOCTYPE HTML><html>"
       << " <head>"
       << "  <title>Pelletsheizung Froeling P2/S3100</title>"
       << "  <meta charset='UTF-8'>"
       << "  <meta name='viewport' content='width=device-width, initial-scale=1.0'>"
       << "  <link rel='icon' href='data:,'>"
       << "</head>"
       << "<body>"
       << "  <h2>ESP Web Server für Pelletsheizung Fröling P2/S3100</h2>"
       << "  <a href='" << (subdirName + targetFilename) << "'>" << (subdirName + targetFilename) << " (" << targetFile.size() << " bytes)   </a>"
       << "  <button type=button><a href='" << (subdirName + targetFilename) << "' download> download</a></button>"
       << "  </br></br>"
       << "  <a href=/>Home</a>"
       << "</body></html>"
       << std::endl;

  return page.str();
}

/*********************************************************************
 * @brief 	configure Webserver
 * @param 	void
 * @return 	void
 *********************************************************************/
void configureWebserver()
{
  // Create AsyncWebServer object on port 80
  static AsyncWebServer server(80);
  static const char index_html[] PROGMEM =
      R"rawliteral(
        <!DOCTYPE HTML><html>
        <head>
          <title>Pelletsheizung Froeling P2/S3100</title>
          <meta charset="UTF-8">
          <meta name="viewport" content="width=device-width, initial-scale=1.0">
          <link rel="icon" href="data:,">
          <style>
          </style>
        </head>
        <body>
          <h2>ESP Web Server für Pelletsheizung Fröling P2/S3100</h2>
          <ul>
            <li><a href=logfiles>Heizungs-Logfiles</a></li>
            <li><a href=sysinfo>ESP32 System Info</a></li>
            <li><a href=syslogfile>System Logfile</a></li>
          </ul>
        <script>
        </script>
        </body>
        </html>
      )rawliteral";

  // Route for root / web page
  server.on(
      "/", HTTP_GET,
      [](AsyncWebServerRequest *request)
      { request->send_P(200, "text/html", index_html); });

  server.serveStatic("/", FILESYSTEM_TO_USE, DATA_DIRECTORY);

  server.onNotFound(
      [](AsyncWebServerRequest *request)
      { request->send(404, "text/plain", "404 - Not found"); });

  server.on(
      "/logfiles", HTTP_GET, [](AsyncWebServerRequest *request)
      { request->send(200, "text/html", handleLogfilesForWebserver(DATA_DIRECTORY).c_str()); });

  server.on(
      "/downloadAllAsOneFile", HTTP_GET, [](AsyncWebServerRequest *request)
      { request->send(200, "text/html", downloadAllAsOneFile(DATA_DIRECTORY).c_str()); });

  server.on(
      "/sysinfo", HTTP_GET, [](AsyncWebServerRequest *request)
      { request->send(200, "text/plain", get_System_Info().c_str()); });

  server.serveStatic("/syslogfile", FILESYSTEM_TO_USE, SYSLOG_FILENAME);

  // before we start the webserver service we have to be sure that WiFi has finished initialisation
  while (WiFi.status() == WL_NO_SHIELD)
  {
    LOG_info << "Wait for WiFi to finished initialisation (WiFi.status() !=255 (WL_NO_SHIELD) ...) -> now WiFi.status()="
             << WiFi.status() << std::endl;
    vTaskDelay(500);
  }
  LOG_info << "Proceed with WiFi.status()=" << WiFi.status() << " -> start webserver" << std::endl;

  // Start server
  server.begin();
  LOG_info << "webserver started" << std::endl;
}
