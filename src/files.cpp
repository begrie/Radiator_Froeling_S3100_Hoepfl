#include "files.h"
#include "debug.h"

#include <list>

/*********************
 * STATIC DEFINITIONS
 *********************/
bool radiator::FilesystemHandler::redirectStdErrToSyslogFile = REDIRECT_STD_ERR_TO_SYSLOG_FILE;
std::string radiator::FilesystemHandler::syslogPathName = SYSLOG_PATHNAME;
std::stringstream radiator::FilesystemHandler::syslogStringStream;
std::ofstream radiator::FilesystemHandler::syslogFileStream;
std::string radiator::FilesystemHandler::message;

/*********************************************************************
 * @brief 	Initialization of filesystem for use with ostream
 * @param 	Filesystem to use: LittleFS (or SPIFFS) or SD
 * @param   Directory for data received from radiator e.g. "/data" or "" for root dir
 * @return 	Pathname to data directory with mountpoint/mountdir like "/littlefs/data"
 *********************************************************************/
std::string
radiator::FilesystemHandler::initFilesystem(std::string dataDirectory)
{
  // std::string message;
  message.reserve(400); // try to avoid heap fragmentation from re-allocation

  if ((void *)&FILESYSTEM_TO_USE == (void *)&SD)
  {
    FILESYSTEM_TO_USE.end();
    if (!FILESYSTEM_TO_USE.begin())
    {
      // retry some more times due to possible problems with sd init
      bool ok = false;
      for (int i = 0; i < 20; i++)
      {
        RADIATOR_LOG_INFO(millis() << " ms: initFilesystem(): retry SD.begin() #" << i << std::endl;)
        delay(200);
        FILESYSTEM_TO_USE.end();
        if (FILESYSTEM_TO_USE.begin())
        {
          RADIATOR_LOG_INFO(millis() << " ms: initFilesystem(): SUCCESS SD.begin() #" << i << std::endl;)
          ok = true;
          break;
        }
      }

      if (!ok)
      {
        message = std::to_string(millis()) + " ms: initFilesystem(): " + XSTRINGIFY(FILESYSTEM_TO_USE) + " MOUNT FAILED!! -> NO LOCAL DATA STORAGE AVAILABLE";
        NetworkHandler::publishToMQTT(message, MQTT_SUBTOPIC_SYSLOG);
        RADIATOR_LOG_ERROR(message << std::endl;)
        return "";
      }
      else
        std::cout << millis() << " ms: initFilesystem() -> SD.begin SUCCESSFULL" << std::endl;
    }
  }
  else // no SD
  {
    if (!FILESYSTEM_TO_USE.begin(false))
    {
      RADIATOR_LOG_INFO(millis() << " ms: initFilesystem(): " << XSTRINGIFY(FILESYSTEM_TO_USE) << " mount failed -> Try to format file system for " << XSTRINGIFY(FILESYSTEM_TO_USE) << " -> please wait ..." << std::endl;)
      if (!FILESYSTEM_TO_USE.begin(true)) // open with format filesystem
      {
        message = std::to_string(millis()) + " ms: initFilesystem(): " + XSTRINGIFY(FILESYSTEM_TO_USE) + " formatting not possible->NO LOCAL DATA STORAGE AVAILABLE ";
        NetworkHandler::publishToMQTT(message, MQTT_SUBTOPIC_SYSLOG);
        RADIATOR_LOG_ERROR(message << std::endl;)
        return "";
      }
      RADIATOR_LOG_INFO(millis() << " ms: initFilesystem(): Formatting of filesystem for " << XSTRINGIFY(FILESYSTEM_TO_USE) << " successfull" << std::endl;)
    }
  }

  // FS_Filehelper::deleteDirectory("/");
  // FILESYSTEM_TO_USE.format();

  RADIATOR_LOG_INFO(millis() << " ms: initFilesystem(): Filesystem   " << XSTRINGIFY(FILESYSTEM_TO_USE)
                             << "   was successfull mounted at basepath:  " << FILESYSTEM_BASE_PATH
                             << "  -> Available space " << ((FILESYSTEM_TO_USE.totalBytes() - FILESYSTEM_TO_USE.usedBytes()) / 1024)
                             << " kB from total " << (FILESYSTEM_TO_USE.totalBytes() / 1024) << " kB" << std::endl;)

  if (!dataDirectory.empty())
  {
    if (dataDirectory.front() != '/')
      dataDirectory = '/' + dataDirectory;

    if (FILESYSTEM_TO_USE.exists((dataDirectory).c_str())) // it seems that LittleFS creates a new dir on its own if it did not exist
    {
      RADIATOR_LOG_INFO(millis() << " ms: initFilesystem(): Directory " << dataDirectory << " already exists" << std::endl;)
    }
    else // create new directory
    {
      RADIATOR_LOG_INFO(millis() << " ms: initFilesystem(): dataDirectory " << dataDirectory << " did not exist -> try to create it ..." << std::endl;)
      if (FILESYSTEM_TO_USE.mkdir(dataDirectory.c_str()))
      {
        RADIATOR_LOG_INFO(millis() << " ms: ... " << dataDirectory << " was successfully created" << std::endl;)
      }
      else
      {
        message = std::to_string(millis()) + " ms: ... " + dataDirectory + " CAN NOT BE CREATED";
        NetworkHandler::publishToMQTT(message, MQTT_SUBTOPIC_SYSLOG);
        RADIATOR_LOG_ERROR(message << std::endl;)

        dataDirectory = "";
      }
    }
  }

  // basepath += dataDirectory;
  dataDirectory = FILESYSTEM_BASE_PATH + dataDirectory;
  RADIATOR_LOG_INFO(millis() << " ms: initFilesystem(): Whole output path is  " << FILESYSTEM_BASE_PATH << std::endl;)

  TaskHandle_t HandlexTaskWatchdogAndMaintenance;

  // Create RTOS task
  BaseType_t _Result = xTaskCreatePinnedToCore(
      xTaskFilesWatchdog,                 // Task function
      "xTaskFilesWatchdog",               // String with name of task
      4096,                               // Stack size in bytes
      NULL,                               // Parameter passed as input of the task
      uxTaskPriorityGet(NULL),            // Priority of the task: higher values -> higher priority
                                          // with uxTaskPriorityGet(NULL)-> same priority as current task
      &HandlexTaskWatchdogAndMaintenance, // Task handle (Typ: TaskHandle_t)
      1);                                 // Core 0 or 1 (Arduino code by default on Core 1)

  if (_Result != pdPASS)
  {
    throw std::runtime_error("xTaskFilesWatchdog: Error creating xTask");
  }

  // FS_Filehelper::deleteDirectory(((std::string)DATA_DIRECTORY).c_str(), FILESYSTEM_TO_USE);
  // createTestData();
  // FS_Filehelper::listDir("/", 255, false, FILESYSTEM_TO_USE);

  return dataDirectory;
}

/**********************************************************************
 *@brief   task for cyclic filesystem actions
 *         -> like filesystem space check
 *         -> close/open of syslog file for write to file
 *@param   pointer to parameter for the task -> nothing here!
 *@return  void
 **********************************************************************/
void radiator::FilesystemHandler::xTaskFilesWatchdog(void *parameter)
{
  vTaskDelay(pdMS_TO_TICKS(30000)); // wait 30 sec. at startup

  const auto intervallForFilesystemCheckSec = INTERVALL_FOR_FILESYSTEM_CHECK_SEC;
  ulong nextFilesystemCheck = 0;

#if REDIRECT_STD_ERR_TO_SYSLOG_FILE
  const auto intervallForWriteSyslogfileSec = WRITE_TO_SYSLOGFILE_INTERVAL_SEC;
  ulong nextWriteSyslogfile = 0;
#else
  const auto intervallForWriteSyslogfileSec = INT_MAX;
#endif

  const auto watchDogIntervallSec = std::min(intervallForFilesystemCheckSec, intervallForWriteSyslogfileSec);

  while (true) // the xTask runs "endless"
  {
    // due to repeated problems with the SD card...
    checkFilesystem();

    if (millis() >= nextFilesystemCheck)
    {
      checkFreeSpaceOnFilesystem();
      nextFilesystemCheck = millis() + intervallForFilesystemCheckSec * 1000;
    }

#if REDIRECT_STD_ERR_TO_SYSLOG_FILE
    if (millis() >= nextWriteSyslogfile)
    {
      writeSyslogfile();
      nextWriteSyslogfile = millis() + intervallForWriteSyslogfileSec * 1000;
    }
#endif

    // DEBUG_STACK_HIGH_WATERMARK

    vTaskDelay((watchDogIntervallSec * 1000) / portTICK_PERIOD_MS); // execute all xx seconds
  }

  assert(false); //-> we must never get here -> if we do -> restart
}

/*********************************************************************
 * @brief 	redirect output from std::err to a syslog file
 * @param 	void
 * @return 	true : success
 *          false: error
 *********************************************************************/
bool radiator::FilesystemHandler::initRedirectStdErrToSyslogFile(std::string_view _syslogPathName)
{
  // std::string inputStr = Serial.readString().c_str();
  // if (!inputStr.empty()) // a key on the console was pressed -> NO redirection until next reboot
  if (!Serial.readString().isEmpty()) // a key on the console was pressed -> NO redirection until next reboot
  {
    std::cout << "\n"
              << millis() << " ms: !!! All output to std::cerr is ONLY SHOWN ON CONSOLE / SERIAL (NO syslog file) -> UNTIL NEXT REBOOT OF ESP" << std::endl;

    return false;
  }

  syslogPathName = _syslogPathName;
  auto dirname = syslogPathName.substr(0, syslogPathName.find('/', 1));
  if (!FILESYSTEM_TO_USE.exists(dirname.c_str()))
  {
    if (FILESYSTEM_TO_USE.mkdir(dirname.c_str()))
      std::cout << millis() << " ms: initRedirectStdErrToSyslogFile(): Created directory " << dirname << std::endl;
    else
      std::cout << millis() << " ms: initRedirectStdErrToSyslogFile(): ERROR creating directory " << dirname << std::endl;
  }

  std::cout << "\n"
            << millis() << " ms: !!! ALL OUTPUT TO std::cerr IS NOW REDIRECTED TO FILE " << syslogPathName << " !!!\n"
            << "\t !!! -> NO OUTPUT TO CONSOLE / SERIAL ANY MORE !!!\n"
            << "\t To deactivate redirection temporarily: hold any key at startup of ESP" << std::endl;

  std::cerr.rdbuf(syslogStringStream.rdbuf()); // redirect by assigning buffer of string stream which is periodically saved to file
  return true;
}

/*********************************************************************
 * @brief 	in case the output to std::err is redirected to a syslog file
 *          -> the stringstream -where std::cerr is redirected to- is written to file
 * @param 	void
 * @return 	void
 *********************************************************************/
bool radiator::FilesystemHandler::writeSyslogfile()
{
#if REDIRECT_STD_ERR_TO_SYSLOG_FILE

  if (syslogStringStream.str().empty()) // Nothing to write
    return false;

  // NetworkHandler::publishToMQTT(syslogStringStream.str(), MQTT_SUBTOPIC_SYSLOG);

  auto file = FILESYSTEM_TO_USE.open(syslogPathName.c_str(), FILE_APPEND);

  // manage syslog files
  if (file && file.size() > MAX_SYSLOG_FILE_SIZE_BYTES)
  {
    RADIATOR_LOG_INFO(millis() << " ms: writeSyslogfile(): syslog file = " << syslogPathName << " (size= " << file.size() << ") reaches max size= " << MAX_SYSLOG_FILE_SIZE_BYTES << std::endl;)

    file.close();

    for (int i = MAX_OLD_SYSLOG_FILES; i >= 0; i--)
    {
      char buf[3];
      snprintf(buf, sizeof(buf), "%02d", i);
      std::string oldSyslogPathname = syslogPathName + ".bak_" + buf;
      if (i == 0)
        oldSyslogPathname = syslogPathName;

      snprintf(buf, sizeof(buf), "%02d", i + 1);
      std::string newSyslogPathname = syslogPathName + ".bak_" + buf;

      if (FILESYSTEM_TO_USE.exists(oldSyslogPathname.c_str()) && i == MAX_OLD_SYSLOG_FILES)
      {
        if (FILESYSTEM_TO_USE.remove(oldSyslogPathname.c_str()))
          RADIATOR_LOG_WARN(millis() << " ms: writeSyslogfile(): Deleted oldest syslog file= " << oldSyslogPathname << std::endl;)
      }
      else
        FILESYSTEM_TO_USE.rename(oldSyslogPathname.c_str(), newSyslogPathname.c_str());
    }

    file = FILESYSTEM_TO_USE.open(syslogPathName.c_str(), FILE_WRITE);
    std::cout << millis() << " ms: writeSyslogfile(): Opened new syslog file= " << syslogPathName << std::endl;
  }

  if (!file)
  {
    std::cout << millis() << " ms: writeSyslogfile(): ERROR opening syslog file = " << syslogPathName << "   !!! System logging now to console:" << std::endl;
    std::cout << syslogStringStream.str() << std::endl;
    return false;
  }

  // write stringstream content to the file
  auto writtenBytes = file.print(syslogStringStream.str().c_str());
  syslogStringStream.str(""); // delete all content from stringstream buffer

  file.close();
  return true;

#endif
  return false;
}

/*********************************************************************
 * @brief 	due to repeated problems with SD: check filesystem for function
 *          -> problems are arised if totalBytes = 0
 * @param 	void
 * @return 	void
 *********************************************************************/
void radiator::FilesystemHandler::checkFilesystem()
{
  if (FILESYSTEM_TO_USE.totalBytes() != 0)
    return;

  std::cout << millis() << " ms: xTaskFilesWatchdog WE HAVE A PROBLEM WITH THE FILESYSTEM: "
            << XSTRINGIFY(FILESYSTEM_TO_USE) << ".totalBytes()= " << FILESYSTEM_TO_USE.totalBytes()
            << " -> we try a re-init ..."
            << std::endl;

  auto res = initFilesystem();

  // std::string message;
  message = std::to_string(millis()) + " ms: .... FILESYSTEM RE-INIT: ";
  // std::cout << millis() << " ms: .... FILESYSTEM RE-INIT: " << (res.empty() ? "FAILED" : "SUCCESS") << " ..." << std::endl;

  static uint8_t problemCounter = 0;

  // make some noise to signal the problem
  if (res.empty())
  {
    problemCounter++;
    message += "FAILED ...";

    pinMode(BUZZER_PIN, OUTPUT);
    bool OnOff = 1;
    for (int i = 0; i < 100; i++)
    {
      digitalWrite(BUZZER_PIN, OnOff);
      OnOff = !OnOff;
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    digitalWrite(BUZZER_PIN, LOW);
  }
  else
  {
    problemCounter = 0;
    message += "SUCCESS ...";
  }

  NetworkHandler::publishToMQTT(message, MQTT_SUBTOPIC_SYSLOG);
  std::cout << message << std::endl;

  if (problemCounter >= 3)
  {
    message = std::to_string(millis()) + " ms: REPEATED PROBLEMS WITH THE FILESYSTEM!! -> NOW RESTART OF THE ESP32";
    NetworkHandler::publishToMQTT(message, MQTT_SUBTOPIC_SYSLOG);
    std::cout << message << std::endl;
    vTaskDelay(pdMS_TO_TICKS(1000)); // wait to deliver message to mqtt
    ESP.restart();
  }
}

/*********************************************************************
 * @brief 	check free space on filesystem
 *          AND REMOVE oldest files if not sufficient
 * @param 	void
 * @return 	void
 *********************************************************************/
void radiator::FilesystemHandler::checkFreeSpaceOnFilesystem()
{
  uint16_t freekBytes = (FILESYSTEM_TO_USE.totalBytes() - FILESYSTEM_TO_USE.usedBytes()) / 1024;

  RADIATOR_LOG_INFO(millis() << " ms: Checking free space limit (" << MIN_FREE_KILOBYTES_ON_FILESYSTEM
                             << " kB) for the logging data on the filesystem: " << XSTRINGIFY(FILESYSTEM_TO_USE) << "\n"
                             << " \t Available " << freekBytes
                             << " kB from total " << (FILESYSTEM_TO_USE.totalBytes() / 1024) << " kB \n";)

  if (freekBytes > MIN_FREE_KILOBYTES_ON_FILESYSTEM)
  {
    RADIATOR_LOG_INFO(" \t -> OK: Enough free space on the filesystem" << std::endl;)
  }
  else
  {
    RADIATOR_LOG_INFO(" \t -> Min. free space limit was undercut -> Oldest logfile in directory "
                          << DATA_DIRECTORY << " will be deleted ..." << std::endl;)

    auto rootDir = FILESYSTEM_TO_USE.open(DATA_DIRECTORY, FILE_READ);
    if (rootDir)
    {
      std::list<std::string> filesList;
      File file = rootDir.openNextFile();
      while (file)
      {
        // RADIATOR_LOG_INFO( "file " << file.path() << std::endl;)
        if (!file.isDirectory())
          filesList.push_back(file.path());

        file = rootDir.openNextFile();
      }

      filesList.sort();
      RADIATOR_LOG_INFO("Sorted files list: \n";)
      for (auto el : filesList)
        RADIATOR_LOG_INFO(el << "\n";)
      RADIATOR_LOG_INFO(std::endl;)

      if (FILESYSTEM_TO_USE.remove(filesList.front().c_str()))
        RADIATOR_LOG_INFO("File   " << filesList.front() << " deleted" << std::endl;)
      else
      {
        // std::string message;
        message = std::to_string(millis()) + " ms: ERROR: xTask_Watchdog_and_Maintenance at Check for free space on filesystem -> FILE  " + filesList.front() + "   CANNOT BE DELETED -> NO SPACE FREED ON FILESYSTEM (" + std::to_string(freekBytes) + " kB available)";
        NetworkHandler::publishToMQTT(message, MQTT_SUBTOPIC_SYSLOG);
        RADIATOR_LOG_ERROR(message << std::endl;)
      }
    }
  }
}

// /*********************************************************************
//  * @brief 	create test data for filesystem check
//  *          -> only for debug
//  * @param 	void
//  * @return 	void
//  *********************************************************************/
// void radiator::FilesystemHandler::createTestData()
// {
//   // FS_Filehelper::deleteDirectory(((std::string)DATA_DIRECTORY).c_str(), FILESYSTEM_TO_USE);

//   FS_Filehelper::writeFile(SYSLOG_PATHNAME,
//                            "Log1 \nLog2 \nLog3 \n",
//                            FILESYSTEM_TO_USE);

//   FS_Filehelper::writeFile(((std::string)DATA_DIRECTORY + "/2021-01-01.log").c_str(),
//                            "11111111111111111\n22222222222222222\n33333333333333\n",
//                            FILESYSTEM_TO_USE);
//   FS_Filehelper::writeFile(((std::string)DATA_DIRECTORY + "/2021-01-02.log").c_str(),
//                            "77777777\n8888888888\n999999999999\n",
//                            FILESYSTEM_TO_USE);

//   FS_Filehelper::writeFile(
//       ((std::string)DATA_DIRECTORY + "/2022-09-13.log").c_str(),
//       "Date; Time;  [S];  [S]; Zustand [N]; ROST [N]; Kesseltemp [N]; Abgastemp. [N]; Abgas. SW  [N]; KessStellGr [N]; Saugzug   [N]; Zuluftgebl [N]; Einschub [N]; Fuellst.: [N]; Feuerraumt [N]; Puffert.ob [N]; Puffert.un [N]; Puffer Pu. [N]; Außentemp [N]; Vorlauft.1sw [N]; Vorlauft.1 [N]; Vorlauft.2sw [N]; Vorlauft.2 [N]; KTY6_H2 [N]; KTY7_H2 [N]; Brenn.ST [N]; Laufzeit: [N]; Boardtemp. [N]; Die Kesseltemp. soll sein [N];\n"
//       "2022-09-13; 10:52:00; Ausgeschaltet ;   Brenner Aus  ; 1; 0; 18°; 23°; 33°; 51%; 0%; 0%; 0%; 99.9%; 394°; 18°; 18°; 0%; 18°; 0°; 20°; 0°; 18°; 127°; 127°; 10461; 15878h; 23°; 75°;\n"
//       "2022-09-13; 10:52:11; Ausgeschaltet ;   Brenner Aus  ; 1; 0; 18°; 23°; 33°; 51%; 0%; 0%; 0%; 99.9%; 394°; 18°; 18°; 0%; 18°; 0°; 20°; 0°; 18°; 127°; 127°; 10461; 15878h; 23°; 75°;\n"
//       "2022-09-13; 10:52:22; Ausgeschaltet ;   Brenner Aus  ; 1; 0; 18°; 23°; 33°; 51%; 0%; 0%; 0%; 99.9%; 394°; 18°; 18°; 0%; 18°; 0°; 20°; 0°; 18°; 127°; 127°; 10461; 15878h; 23°; 75°;\n",
//       FILESYSTEM_TO_USE);

//   FS_Filehelper::writeFile(
//       ((std::string)DATA_DIRECTORY + "/2022-09-12.log").c_str(),
//       "Date; Time;  [S];  [S]; Zustand [N]; ROST [N]; Kesseltemp [N]; Abgastemp. [N]; Abgas. SW  [N]; KessStellGr [N]; Saugzug   [N]; Zuluftgebl [N]; Einschub [N]; Fuellst.: [N]; Feuerraumt [N]; Puffert.ob [N]; Puffert.un [N]; Puffer Pu. [N]; Außentemp [N]; Vorlauft.1sw [N]; Vorlauft.1 [N]; Vorlauft.2sw [N]; Vorlauft.2 [N]; KTY6_H2 [N]; KTY7_H2 [N]; Brenn.ST [N]; Laufzeit: [N]; Boardtemp. [N]; Die Kesseltemp. soll sein [N];\n"
//       "2022-09-12; 10:52:00; Ausgeschaltet ;   Brenner Aus  ; 1; 0; 18°; 23°; 33°; 51%; 0%; 0%; 0%; 99.9%; 394°; 18°; 18°; 0%; 18°; 0°; 20°; 0°; 18°; 127°; 127°; 10461; 15878h; 23°; 75°;\n"
//       "2022-09-12; 10:52:11; Ausgeschaltet ;   Brenner Aus  ; 1; 0; 18°; 23°; 33°; 51%; 0%; 0%; 0%; 99.9%; 394°; 18°; 18°; 0%; 18°; 0°; 20°; 0°; 18°; 127°; 127°; 10461; 15878h; 23°; 75°;\n"
//       "2022-09-12; 10:52:22; Ausgeschaltet ;   Brenner Aus  ; 1; 0; 18°; 23°; 33°; 51%; 0%; 0%; 0%; 99.9%; 394°; 18°; 18°; 0%; 18°; 0°; 20°; 0°; 18°; 127°; 127°; 10461; 15878h; 23°; 75°;\n",
//       FILESYSTEM_TO_USE);

//   FS_Filehelper::writeFile(
//       ((std::string)DATA_DIRECTORY + "/2022-08-22.log").c_str(),
//       "Date; Time;  [S];  [S]; Zustand [N]; ROST [N]; Kesseltemp [N]; Abgastemp. [N]; Abgas. SW  [N]; KessStellGr [N]; Saugzug   [N]; Zuluftgebl [N]; Einschub [N]; Fuellst.: [N]; Feuerraumt [N]; Puffert.ob [N]; Puffert.un [N]; Puffer Pu. [N]; Außentemp [N]; Vorlauft.1sw [N]; Vorlauft.1 [N]; Vorlauft.2sw [N]; Vorlauft.2 [N]; KTY6_H2 [N]; KTY7_H2 [N]; Brenn.ST [N]; Laufzeit: [N]; Boardtemp. [N]; Die Kesseltemp. soll sein [N];\n"
//       "2022-08-22; 10:52:00; Ausgeschaltet ;   Brenner Aus  ; 1; 0; 18°; 23°; 33°; 51%; 0%; 0%; 0%; 99.9%; 394°; 18°; 18°; 0%; 18°; 0°; 20°; 0°; 18°; 127°; 127°; 10461; 15878h; 23°; 75°;\n"
//       "2022-08-22; 10:52:11; Ausgeschaltet ;   Brenner Aus  ; 1; 0; 18°; 23°; 33°; 51%; 0%; 0%; 0%; 99.9%; 394°; 18°; 18°; 0%; 18°; 0°; 20°; 0°; 18°; 127°; 127°; 10461; 15878h; 23°; 75°;\n"
//       "2022-08-22; 10:52:22; Ausgeschaltet ;   Brenner Aus  ; 1; 0; 18°; 23°; 33°; 51%; 0%; 0%; 0%; 99.9%; 394°; 18°; 18°; 0%; 18°; 0°; 20°; 0°; 18°; 127°; 127°; 10461; 15878h; 23°; 75°;\n",
//       FILESYSTEM_TO_USE);
// }
