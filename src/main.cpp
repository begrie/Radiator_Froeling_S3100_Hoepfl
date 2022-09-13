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
#define FILESYSTEM_TO_USE LittleFS           // LittleFS (or SPIFFS) or SD
#define DATA_DIRECTORY "/P2_Logging"         // directory for radiator data -> e.g. "/logging" or "" for root dir of used filesystem
#define MIN_FREE_KILOBYTES_ON_FILESYSTEM 300 // if limit is undercut -> oldest logfiles are deleted

// !!! SEE MORE #defines in output.h. FOR CONTROLLING THE OUTPUT BEHAVIOUR !!!

/********************************************
 * FORWARD DECLARATIONS
 ********************************************/
void xTask_Watchdog_and_Maintenance(void *parameter);
std::string initFilesystem(std::string dataDirectory);
void checkFreeSpaceOnFilesystem();
void writeSyslogfile();

/********************************************
 * GLOBAL DEFINITIONS
 ********************************************/
std::string pathnameToDataDirectory; // with mountpoint/mountdir of filesystem
bool outputToConsole = OUTPUT_TO_CONSOLE;
bool outputToMQTT = OUTPUT_TO_MQTT;

#if REDIRECT_STD_ERR_TO_SYSLOG_FILE
std::ofstream syslogFileStream;
std::string syslogFileName = "/littlefs/syslog1.log";
#endif

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

  if (!OUTPUT_TO_FILE)
    pathnameToDataDirectory = ""; // empty pathname indicates "no file output" for constuctor of OutputHandler
  else
  {
    pathnameToDataDirectory = initFilesystem(dataDirectory); // LittleFS (or SPIFFS or SD), directory for data
                                                             // patnameToFIles is completed with mountpoint of the filesystem
    if (pathnameToDataDirectory.empty())
    {
      std::cout << "ERROR MOUNTING FILESYSTEM !!! -> Instead: data output to console (std::cout)" << std::endl;
      outputToConsole = true;
    }
    else
    {
#if REDIRECT_STD_ERR_TO_SYSLOG_FILE
      // Ausgaben auf Logfile umleiten -> geht so nur mit regelmäßigem close und re-open des streams -> BUG bei LittleFS??
      LOG_info << millis() << " ms: !!! ALL DEBUG OUTPUT TO std::err IS NOW REDIRECTED TO FILE " << syslogFileName << " !!!\n"
               << "  \t !!! -> NO OUTPUT TO CONSOLE ANY MORE !!!" << std::endl;
      syslogFileStream.open(syslogFileName, std::ofstream::out | std::ofstream::app);
      if (syslogFileStream && syslogFileStream.is_open() && syslogFileStream.good())
      {
        //  syslogFilestream.rdbuf()->pubsetbuf(0, 0);  //sollte ohne buffer direkt schreiben -> kein Wirkung bei littlefs
        syslogFileStream << "Syslogfile " << syslogFileName << " opened successfull " << std::endl;
        std::cerr.rdbuf(syslogFileStream.rdbuf()); // redirect by assigning buffer of file stream
                                                   // std::cerr.tie(&syslogFilestream);
        syslogFileStream << "Redirect output from std::err to syslogfile " << std::endl;
      }
      else
      {
        LOG_info << "ERROR opening syslog file " << syslogFileName << std::endl;
      }
      LOG_info << "NACH       syslogFilestream.open('/littlefs/syslog.log');" << std::endl;
#endif

      TaskHandle_t Handle_xTask_Watchdog_and_Maintenance;

      // Create RTOS task
      BaseType_t _Result = xTaskCreatePinnedToCore(
          xTask_Watchdog_and_Maintenance,         // Task function
          "xTask_Watchdog_and_Maintenance",       // String with name of task
          4096,                                   // Stack size in bytes
          NULL,                                   // Parameter passed as input of the task
          uxTaskPriorityGet(NULL),                // Priority of the task: higher values -> higher priority
                                                  // with uxTaskPriorityGet(NULL)-> same priority as current task
          &Handle_xTask_Watchdog_and_Maintenance, // Task handle (Typ: TaskHandle_t)
          1);                                     // Core 0 or 1 (Arduino code by default on Core 1)

      if (_Result != pdPASS)
      {
        throw std::runtime_error("Start_Watchdog_and_Maintenance_Task: Error creating xTask");
      }
    }
  }
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
    // for (;;)
    // {
    LOG_trace << "Starting main loop" << std::endl;

    radiator::OutputHandler outHandler(pathnameToDataDirectory, // with empty pathnameToDataDirectory -> no output to file
                                       outputToConsole,
                                       outputToMQTT);
    radiator::Surveillance surveillance(S_SERIAL_TO_RADIATOR, T_TIMEOUT_BETWEEN_TRANSFERS_MS, outHandler);
    surveillance.main_loop();

    LOG_info << "Main loop ended, restarting in 10s" << std::endl;
    sleep(10);

    // }
  }
  catch (const char *&error)
  {
    LOG_fatal << "Failed to open serial device or filesystem -> restarting ESP32 in 10s" << std::endl;
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
void xTask_Watchdog_and_Maintenance(void *parameter)
{
  vTaskDelay(pdMS_TO_TICKS(10000)); // wait 10 sec. at startup

  const auto intervallForFilesystemCheckSec = 6 * 60 * 60; // execute all 6 hours
  ulong nextFilesystemCheck = 0;

#if REDIRECT_STD_ERR_TO_SYSLOG_FILE
  const auto intervallForWriteSyslogfileSec = 15; // all 15 minutes
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
    basepath = "/littlefs"; // standard mountpoint(mountpath)
  }
  else if ((void *)&FILESYSTEM_TO_USE == (void *)&SD)
  {
    fsName = "SD";
    basepath = "/sd"; // standard mountpoint(mountpath)
  }
  else if ((void *)&FILESYSTEM_TO_USE == (void *)&SPIFFS)
  {
    fsName = "SPIFFS";
    basepath = "/spiffs"; // standard mountpoint(mountpath)
  }

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

  FS_Filehelper::listDir("/", 255, true, FILESYSTEM_TO_USE);

  return basepath;
}

/*********************************************************************
 * @brief 	Initialization of filesystem for use with ostream
 * @param 	Filesystem to use: LittleFS or SPIFFS or SD
 * @param   Directory for data received from radiator e.g. "/data" or "" for root dir
 * @return 	Pathname to data directory with mountpoint/mountdir like "/littlefs/data"
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
    syslogFileStream.open(syslogFileName, std::ofstream::out | std::ofstream::app);
  }
#endif
}
