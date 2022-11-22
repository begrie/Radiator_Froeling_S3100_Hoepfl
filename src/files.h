#ifndef __DH_FILES_H__
#define __DH_FILES_H__

#include <fstream>

#include <LittleFS.h>
#include <SPIFFS.h>
#include <SD.h>

#include "config.h"
#include "network.h"
#include "FS_Filehelper.h" //only for testing

namespace radiator
{
  class FilesystemHandler
  {
  protected:
    static void xTaskFilesWatchdog(void *parameter);

    static bool writeSyslogfile();

    static bool redirectStdErrToSyslogFile;
    static std::string syslogPathName;
    static std::stringstream syslogStringStream;
    static std::ofstream syslogFileStream;

    static void checkFilesystem();
    static void checkFreeSpaceOnFilesystem();
    static void createTestData();

  public:
    static std::string initFilesystem(std::string dataDirectory = DATA_DIRECTORY);
    static bool initRedirectStdErrToSyslogFile(std::string _syslogPathName = SYSLOG_PATHNAME);
  };
} // namespace radiator
#endif //#ifndef __DH_FILES_H__