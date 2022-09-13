/******************************************************************************
 * SPIFFS_Filehelper.h
 * @brief    Libs und Funktionen zum ESP32-Dateihandling (im internen Flash-Speicher SPIFFS)
 *           -> https://techtutorialsx.com/2018/08/05/esp32-arduino-spiffs-writing-a-file/
 *           BG 2018
 ******************************************************************************/

#ifndef _FS_FILEHELPER_h
#define _FS_FILEHELPER_h

/*********************
 *      DEBUG
 *********************/
//#define DEBUG_FS_FILEHELPER_h // definieren -> wenn Debugausgaben gew�nscht; auskommentieren -> wenn nicht
#if defined DEBUG_FS_FILEHELPER_h
#define DEBUG_MESSAGE_FS_FILEHELPER_h(format, ...) Serial.printf("%lu ms: DEBUG %s::%s(%d): " format "\n", millis(), __FILE__, __FUNCTION__, __LINE__, __VA_ARGS__)
// -> Usage: DEBUG_MESSAGE_FS_FILEHELPER_h("int1=%d, cString=%s, stdString=%s", int1, cString, stdString.c_str());
#else                                              // KEINE Debugausgaben ...
#define DEBUG_MESSAGE_FS_FILEHELPER_h(format, ...) //... im Ergebnis leeres #define ....
#endif

/*********************
 *      MAKROS
 *********************/
// angereichertes Makro als Ersatz für Serial.printf(...)
// -> Usage: SERPRINT("int1=%d, cString=%s, stdString=%s", int1, cString, stdString.c_str());
#define SERIALPRINT(format, ...) Serial.printf("%lu ms: %s:%d %s : " format "\n", millis(), __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__);

/*********************
 *      DEFINES
 *********************/

/*********************
 *      INCLUDES
 *********************/
#include <Arduino.h>
#include <FS.h>
#include <LITTLEFS.h>
// #include <SPIFFS.h>
#include <SD.h>

/*********************
 *      DECLARATIONS
 *********************/
class FS_Filehelper
{
public:
  static bool listDir(const char *dirname = "/", uint8_t levels = 255, bool with_content = false, fs::FS &fs = LittleFS);
  static bool readFile(const char *path, fs::FS &fs = LittleFS);
  static bool writeFile(const char *path, const char *message, fs::FS &fs = LittleFS);
  static bool appendFile(const char *path, const char *message, fs::FS &fs = LittleFS);
  static bool deleteFile(const char *path, fs::FS &fs = LittleFS);                            // only for files -> no directories
  static bool deleteDirectory(const char *path,  fs::FS &fs = LittleFS); // recursive with files
};

#endif // von #ifndef _FS_FILEHELPER_h
