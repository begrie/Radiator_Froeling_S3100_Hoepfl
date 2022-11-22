#include "files.h"
#include "debug.h"

#include <list>

bool radiator::FilesystemHandler::redirectStdErrToSyslogFile = REDIRECT_STD_ERR_TO_SYSLOG_FILE;
std::string radiator::FilesystemHandler::syslogPathName = SYSLOG_PATHNAME;
std::stringstream radiator::FilesystemHandler::syslogStringStream;
std::ofstream radiator::FilesystemHandler::syslogFileStream;

/*********************************************************************
 * @brief 	Initialization of filesystem for use with ostream
 * @param 	Filesystem to use: LittleFS (or SPIFFS) or SD
 * @param   Directory for data received from radiator e.g. "/data" or "" for root dir
 * @return 	Pathname to data directory with mountpoint/mountdir like "/littlefs/data"
 *********************************************************************/
std::string radiator::FilesystemHandler::initFilesystem(std::string dataDirectory)
{
  std::string basepath;
  std::string fsName;

  if ((void *)&FILESYSTEM_TO_USE == (void *)&SD)
  {
    fsName = "SD";
    basepath = "/sd"; // standard mountpoint(mountpath)
    FILESYSTEM_TO_USE.end();
    if (!FILESYSTEM_TO_USE.begin())
    {
      // retry some more times due to possible problems with sd init
      bool ok = false;
      for (int i = 0; i < 20; i++)
      {
        LOG_info << millis() << " ms: retry SD.begin() #" << i << std::endl;
        delay(200);
        FILESYSTEM_TO_USE.end();
        if (FILESYSTEM_TO_USE.begin())
        {
          LOG_info << millis() << " ms: SUCCESS SD.begin() #" << i << std::endl;
          ok = true;
          break;
        }
      }

      if (!ok)
      {
        std::stringstream message;
        message << millis() << " ms: " << fsName << " MOUNT FAILED!! -> NO LOCAL DATA STORAGE AVAILABLE" << std::endl;
        NetworkHandler::publishToMQTT(message.str(), MQTT_TOPIC_SYSLOG);
        LOG_error << message.str();
        return "";
      }
      else
        std::cout << millis() << " ms: initFilesystem() -> SD.begin SUCCESSFULL" << std::endl;
    }
  }
  else // no SD
  {
    if ((void *)&FILESYSTEM_TO_USE == (void *)&LittleFS)
    {
      fsName = "LittleFS";
      basepath = "/littlefs"; // standard mountpoint(mountpath)
    }
    else if ((void *)&FILESYSTEM_TO_USE == (void *)&SPIFFS)
    {
      fsName = "SPIFFS";
      basepath = "/spiffs"; // standard mountpoint(mountpath)
    }
    if (!FILESYSTEM_TO_USE.begin(false))
    {
      LOG_info << fsName << " mount failed -> Try to format file system for " << fsName << " -> please wait ..." << std::endl;
      if (!FILESYSTEM_TO_USE.begin(true)) // open with format filesystem
      {
        std::stringstream message;
        message << fsName << " formatting not possible -> NO LOCAL DATA STORAGE AVAILABLE" << std::endl;
        NetworkHandler::publishToMQTT(message.str(), MQTT_TOPIC_SYSLOG);
        LOG_error << message.str();
        return "";
      }
      LOG_info << "Formatting of filesystem for " << fsName << " successfull" << std::endl;
    }
  }

  // FS_Filehelper::deleteDirectory("/");
  // FILESYSTEM_TO_USE.format();

  LOG_info << millis() << " ms: Filesystem   " << fsName << "   was successfull mounted at basepath:  " << basepath
           << "  -> Available space " << ((FILESYSTEM_TO_USE.totalBytes() - FILESYSTEM_TO_USE.usedBytes()) / 1024)
           << " kB from total " << (FILESYSTEM_TO_USE.totalBytes() / 1024) << " kB" << std::endl;

  if (!dataDirectory.empty())
  {
    if (dataDirectory.front() != '/')
      dataDirectory = '/' + dataDirectory;

    if (FILESYSTEM_TO_USE.exists((dataDirectory).c_str())) // it seems that LittleFS creates a new dir on its own if it did not exist
    {
      LOG_info << millis() << " ms: Directory " << dataDirectory << " already exists" << std::endl;
    }
    else // create new directory
    {
      LOG_info << millis() << " ms: dataDirectory " << dataDirectory << " did not exist -> try to create it ..." << std::endl;
      if (FILESYSTEM_TO_USE.mkdir(dataDirectory.c_str()))
      {
        LOG_info << millis() << " ms: ... " << dataDirectory << " was successfully created" << std::endl;
      }
      else
      {
        std::stringstream message;
        message << millis() << " ms: ... " << dataDirectory << " CAN NOT BE CREATED" << std::endl;
        NetworkHandler::publishToMQTT(message.str(), MQTT_TOPIC_SYSLOG);
        LOG_error << message.str();

        dataDirectory = "";
      }
    }
  }

  basepath += dataDirectory;
  LOG_info << millis() << " ms: Whole output path is  " << basepath << std::endl;

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

  return basepath;
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
  vTaskDelay(pdMS_TO_TICKS(10000)); // wait 10 sec. at startup

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
bool radiator::FilesystemHandler::initRedirectStdErrToSyslogFile(std::string _syslogPathName)
{
  std::string inputStr = Serial.readString().c_str();
  if (!inputStr.empty()) // a key on the console was pressed -> NO redirection until next reboot
  {
    LOG_info << "\n"
             << millis() << " ms: !!! All output to std::cerr is ONLY SHOWN ON CONSOLE / SERIAL (NO syslog file) -> UNTIL NEXT REBOOT OF ESP" << std::endl;

    return false;
  }

  syslogPathName = _syslogPathName;
  auto dirname = syslogPathName.substr(0, syslogPathName.find('/', 1));
  if (!FILESYSTEM_TO_USE.exists(dirname.c_str()))
  {
    if (FILESYSTEM_TO_USE.mkdir(dirname.c_str()))
      std::cout << "Created directory " << dirname << std::endl;
    else
      std::cout << "ERROR creating directory " << dirname << std::endl;
  }

  LOG_info
      << "\n"
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

  // NetworkHandler::publishToMQTT(syslogStringStream.str(), MQTT_TOPIC_SYSLOG);

  auto file = FILESYSTEM_TO_USE.open(syslogPathName.c_str(), FILE_APPEND);

  // manage syslog files
  if (file && file.size() > MAX_SYSLOG_FILE_SIZE_BYTES)
  {
    LOG_info << millis() << " ms: syslog file = " << syslogPathName << " (size= " << file.size() << ") reaches max size= " << MAX_SYSLOG_FILE_SIZE_BYTES << std::endl;

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
      // std::string oldSyslogPathname = syslogPathName + ".bak_" + std::to_string(i);
      // if (i == 0)
      //   oldSyslogPathname = syslogPathName;

      // std::string newSyslogPathname = syslogPathName + ".bak_" + std::to_string(i + 1);

      // std::cout << millis() << " ms: oldSyslogPathname=" << oldSyslogPathname << ", newSyslogPathname=" << newSyslogPathname << std::endl;

      if (FILESYSTEM_TO_USE.exists(oldSyslogPathname.c_str()) && i == MAX_OLD_SYSLOG_FILES)
      {
        if (FILESYSTEM_TO_USE.remove(oldSyslogPathname.c_str()))
          LOG_warn << millis() << " ms: Deleted oldest syslog file= " << oldSyslogPathname << std::endl;
      }
      else
        FILESYSTEM_TO_USE.rename(oldSyslogPathname.c_str(), newSyslogPathname.c_str());
    }

    file = FILESYSTEM_TO_USE.open(syslogPathName.c_str(), FILE_WRITE);
    std::cout << millis() << " ms: Opened new syslog file= " << syslogPathName << std::endl;
  }

  if (!file)
  {
    std::cout << millis() << " ms: ERROR opening syslog file = " << syslogPathName << "   !!! System logging now to console:" << std::endl;
    std::cout << syslogStringStream.str() << std::endl;
    return false;
  }

  // write stringstream content to the file
  auto writtenBytes = file.print(syslogStringStream.str().c_str());
  syslogStringStream.str(""); // delete all content

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

  std::stringstream message;
  message << millis() << " ms: .... FILESYSTEM RE-INIT: ";
  // std::cout << millis() << " ms: .... FILESYSTEM RE-INIT: " << (res.empty() ? "FAILED" : "SUCCESS") << " ..." << std::endl;

  static uint8_t problemCounter = 0;

  // make some noise to signal the problem
  if (res.empty())
  {
    problemCounter++;
    message << "FAILED ..." << std::endl;

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
    message << "SUCCESS ..." << std::endl;
  }

  NetworkHandler::publishToMQTT(message.str(), MQTT_TOPIC_SYSLOG);
  std::cout << message.str();

  if (problemCounter >= 3)
  {
    message << millis() << " ms: REPEATED PROBLEMS WITH THE FILESYSTEM!! -> NOW RESTART OF THE ESP32" << std::endl;
    NetworkHandler::publishToMQTT(message.str(), MQTT_TOPIC_SYSLOG);
    std::cout << message.str();
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
  auto freekBytes = (FILESYSTEM_TO_USE.totalBytes() - FILESYSTEM_TO_USE.usedBytes()) / 1024;

  LOG_info
      << millis() << " ms: Checking free space limit (" << MIN_FREE_KILOBYTES_ON_FILESYSTEM
      << " kB) for the logging data on the filesystem:\n"
      << " \t Available " << freekBytes
      << " kB from total " << (FILESYSTEM_TO_USE.totalBytes() / 1024) << " kB \n"
      << std::endl;

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
      {
        std::stringstream message;
        message << millis() << " ms: ERROR: xTask_Watchdog_and_Maintenance at Check for free space on filesystem -> FILE  "
                << filesList.front()
                << "   CANNOT BE DELETED -> NO SPACE FREED ON FILESYSTEM (" << freekBytes << " kB available)" << std::endl;
        NetworkHandler::publishToMQTT(message.str(), MQTT_TOPIC_SYSLOG);
        LOG_error << message.str();
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
