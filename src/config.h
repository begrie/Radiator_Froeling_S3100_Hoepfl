/******************************************************************************
 * Name:		config.h
 * @brief   Konfiguration für Überwachungstool für Fröling P2 / Lambdatronic S 3100
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
#ifndef __DH_CONFIG_H__
#define __DH_CONFIG_H__

/*********************
 *      INCLUDES
 *********************/
#include <Arduino.h>
#include <LittleFS.h>
#include <SPIFFS.h>
#include <SD.h>

namespace radiator
{

/*********************
 *      DEFINES
 *********************/
#define D_DEBUG_LEVEL 2 // 0 - No logging
                        // 1 - Errors
                        // 2 - Warnings
                        // 3 - Infos
                        // 4 - Debug
                        // 5 - Trace

#define REDIRECT_STD_ERR_TO_SYSLOG_FILE false    // debug output to file INSTEAD to console/Serial
                                                 // -> do it only with D_DEBUG_LEVEL 0, 1 or 2
#define SYSLOG_DIR "/syslog"                     // directory for syslog files
#define SYSLOG_PATHNAME SYSLOG_DIR "/syslog.log" // name with path (only one directory level is created automatic)
                                                 // -> watch the needed filespace: makes only sense at long term operation
                                                 //    with D_DEBUG_LEVEL 1 - Errors (or 2 -Warnings)
#define WRITE_TO_SYSLOGFILE_INTERVAL_SEC 60      // controls how often the streambuffer of the syslogfile
                                                 // is written to file and how many data can get lost ... (e.g. 60  -> 60 sec are a good value)
#define MAX_SYSLOG_FILE_SIZE_BYTES (100 * 1024)  // there may be problems with serving too big files (>30 kB?) from AsyncWebserver
#define MAX_OLD_SYSLOG_FILES 15

#define S_SERIAL_TO_RADIATOR "Serial2" // Serial device for connection to radiator device P2/S3100
// #define SERIAL_TO_RADIATOR_RX 16 // ESP32-standard: 16
// #define SERIAL_TO_RADIATOR_TX 17 // ESP32-standard: 17
#define T_TIMEOUT_BETWEEN_TRANSFERS_MS 5000 // Maximum time allowed between transfers from P2/S3100 radiator device
                                            // until connection is considered stale.

#define USE_WIFI true
#define WIFI_HOSTNAME "Heizung_Froeling_P2"
#define START_WEBSERVER false // needs activated WiFi (USE_WIFI true)
// #define NUMBER_OF_LOGFILES_TO_COMBINE 92 // download option for webserver: 3 month -> approx 92 day files

// Where to write the values received from the radiator to:
#define OUTPUT_TO_CONSOLE false
#define OUTPUT_TO_MQTT true // needs activated WiFi
#define OUTPUT_TO_FILE true

#if !USE_WIFI
#define START_WEBSERVER false // needs activated WiFi
#define OUTPUT_TO_MQTT false  // needs activated WiFi
#endif

#define FILESYSTEM_TO_USE SD       // SD  (or  LittleFS  (or SPIFFS)) -> errors are NOT HANDLED if filesystem not exists (or SD card not inserted)
#define FILESYSTEM_BASE_PATH "/sd" // "/littlefs" or "/sd" (or "/spiffs") -> MUST CORRESPOND to above filesystem definition
// #define FILESYSTEM_TO_USE LittleFS           // LittleFS or SD (or SPIFFS)
// #define FILESYSTEM_BASE_PATH "/littlefs"     // "/littlefs" or "/sd" (or "/spiffs") -> must correspond to above filesystem definition
#define DATA_DIRECTORY "/P2_Logging"                      // DATA_DIRECTORY: directory for data from radiator -> e.g. "/data" or "" for root dir of used filesystem
                                                          // only one level is created automatically in initFilesystem()
                                                          // ->all deeper directories in path must exist before instantiation of OutputHandler
#define MIN_FREE_KILOBYTES_ON_FILESYSTEM 5000             // if limit is undercut -> oldest logfiles are deleted
#define INTERVALL_FOR_FILESYSTEM_CHECK_SEC (24 * 60 * 60) // execute all 24 hours to check free space on filesystem

#define FILE_OUTPUT_INTERVALL_SEC 60   // controls how often resp. how many of the received data is saved to a logfile
                                       // -> the radiator device sends every second one values data set and the
                                       //    filtering is made by dropping (standard) or averaging (not yet implemented)
                                       //    the values from the previous time series
                                       // -> one values data set needs ca. 200 bytes space in a csv file
                                       //    -> with 60 sec FILE_OUTPUT_INTERVALL_SEC ca. 280kB per day
                                       //       -> ca. 100MB per year (needs SD card output)
                                       //    -> with flash filesystem like littlefs ca. 1,4 to 2 MB available
                                       //       3,8kB to 5,5kB per day allowed to store one years data -> only ca. 1 value per hour
                                       //       -> with 5min(300sec) intervall -> storage for 24 to 34 days
#define WRITE_TO_FILE_INTERVAL_SEC 300 // controls how often the streambuffer of the logfile
                                       // is written to file and how many data can get lost ...
                                       // the greater the value the lower the stress of the flash or SD card
                                       // 30 for testing; later 300 = 5 min for 1 values item per minute

#define DELIMITER_FOR_CSV_FILE "; "

#define MQTT_BROKER "broker.hivemq.com" //"broker.emqx.io"  //"broker.hivemq.com"
// USE OF IP NOT IMPLEMENTED #define MQTT_BROKER_IP IPAddress(18, 196, 225, 29) //"broker.hivemq.com"
#define MQTT_USER ""     // broker.hivemq.com works without credentials
#define MQTT_PASSWORD "" // broker.hivemq.com works without credentials
#define MQTT_PORT 1883
#define MQTT_OUTPUTINTERVALL_SEC 60 // controls how often resp. how many of the received
                                    // data is send to a MQTT broker
#define MQTT_TOPIC WIFI_HOSTNAME    // use WiFi hostname as basic mqtt topic
#define MQTT_SUBTOPIC_ONLINESTATUS "/onlinestatus"
#define MQTT_SUBTOPIC_SYSLOG "/syslog"
#define MQTT_SUBTOPIC_ERRORLOG "/errorlog"
#define MQTT_SUBTOPIC_ANAYLYSIS "/analysis"
#define MQTT_SUBTOPIC_SYSINFO "/systeminfo"
#define MQTT_INTERVALL_FOR_SYSINFO_SEC (1 * 60 * 60)       //(1 * 60 * 60)       // execute all 1 hours to send systeminfo to mqtt broker
#define MQTT_KEEP_ALIVE (MQTT_OUTPUTINTERVALL_SEC * 2.5)   // must be valid together with MQTT_OUTPUTINTERVALL_SEC
#define MQTT_RECONNECTION_TIMEOUT_SEC 15                   // timeout after disconnection from MQTT broker
                                                           // until a reconnection is tried from ESP
#define MIN_FREE_HEAPSIZE_FOR_MQTT_BUFFERQUEUE_BYTES 25000 // MQTT messages are buffered for resending after conditional
                                                           // broker disconnection ->  if this free heap size value is undercut
                                                           // -> the oldest buffered values are dropped

#define BUZZER_PIN 27
#define QUIT_BUZZER_BUTTON_PIN 13
#define BEEP_INTERVALL_RADIATOR_ERROR_MS 700

#define PREFERENCES_NAMESPACE "FroelingP2" // limited to 15 chars

#define USE_EXTERNAL_SENSORS true // use or disable functionality from externalsensors.h/.cpp
#define GPIO_FOR_DHT11 21         // GPIO 21    or   0 to deactivate
#define DHTTYPE DHT11             // use type definition from DHT lib

#define GPIO_FOR_VENTILATOR_RELAIS 22                                // GPIO 22  or 0 to deactivate
#define MIN_TEMP_FOR_VENTILATOR_ON 26                                // °C
#define MAX_TEMP_FOR_VENTILATOR_OFF (MIN_TEMP_FOR_VENTILATOR_ON - 2) // °C; hysterisis to avoid nervous switching
                                                                     // due to measurement fluctuations
#define MAX_HUMIDITY_FOR_VENTILATOR_RUN 25                           // %

#define GPIO_FOR_SERVO_FOR_AIR_INPUT_FLAP 25              // GPIO 25  or  0 to deactivate
#define CLOSED_ANGLE_FOR_SERVO_FOR_AIR_INPUT_FLAP 45      // °   angle in degree
#define OPENED_ANGLE_FOR_SERVO_FOR_AIR_INPUT_FLAP 135     // °   angle in degree
#define DEGREE_PER_SECOND_FOR_SERVO_FOR_AIR_INPUT_FLAP 60 // speed of servo movement

#define GPIO_FOR_LEAKWATER_SENSOR 39 // GPIO 39   or   0 to deactivate

#define GPIO_FOR_AC_CURRENT_SENSOR 33                                // GPIO 33    or 0 to deactivate
#define AC_CURRENT_SENSOR_SCALE_AMPERE_PER_MILLIVOLT (30.0 / 1000.0) // e.g. for 30 Ampere per 1000 Millivolt

#define ACTIVATE_ANALYSIS true

/*********************
 *      MACROS
 *********************/
#define XSTRINGIFY(a) STRINGIFY(a)
#define STRINGIFY(a) #a

} // namespace radiator

#endif // #ifndef __DH_CONFIG_H__
