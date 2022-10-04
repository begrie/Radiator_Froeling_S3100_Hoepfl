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

namespace radiator
{

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

#define USE_WIFI true
#define WIFI_HOSTNAME "Heizung_Froeling_P2"
#define START_WEBSERVER true             // needs activated WiFi
#define NUMBER_OF_LOGFILES_TO_COMBINE 92 // download option for webserver: 3 month -> approx 92 day files

// Where to write the received values to:
#define OUTPUT_TO_CONSOLE false
#define OUTPUT_TO_MQTT true // needs activated WiFi
#define OUTPUT_TO_FILE true

#if !USE_WIFI
#define START_WEBSERVER false // needs activated WiFi
#define OUTPUT_TO_MQTT false  // needs activated WiFi
#endif

#define FILESYSTEM_TO_USE LittleFS           // LittleFS or SD (or SPIFFS)
#define FILESYSTEM_BASE_PATH "/littlefs"     // "/littlefs" or "/sd" (or "/spiffs") -> must correspond to above filesystem definition
#define DATA_DIRECTORY "/P2_Logging"         // directory for radiator data -> e.g. "/logging" or "" for root dir of used filesystem
#define MIN_FREE_KILOBYTES_ON_FILESYSTEM 300 // if limit is undercut -> oldest logfiles are deleted
#define FILE_OUTPUT_INTERVALL_SEC 10         // controls how often resp. how many of the received data is saved to a logfile
                                             // -> the radiator device sends every second one values data set
                                             //    and the filtering is made by dropping (standard) or averaging (not yet implemented)
                                             //    the values from the previous time series
                                             // -> one values data set needs ca. 200 bytes space in a csv file
                                             //    -> with 60 sec FILE_OUTPUT_INTERVALL_SEC ca. 280kB per day
                                             //       -> ca. 100MB per year (needs SD card output)
                                             //    -> with flash filesystem like littlefs ca. 1,4 to 2 MB available
                                             //       3,8kB to 5,5kB per day allowed to store one years data -> only ca. 1 value per hour
                                             //       -> with 5min(300sec) intervall -> storage for 24 to 34 days
#define WRITE_TO_FILE_INTERVAL_SEC 30        // controls how often the streambuffer of the logfile
                                             // is written to file and how many data can get lost ...
                                             // the greater the value the lower the stress of the flash or SD card
                                             // 30 for testing; later 900 = 15 min for 1 values item per minute
#define DELIMITER_FOR_CSV_FILE "; "

#define MQTT_BROKER "broker.hivemq.com" //"broker.emqx.io"  //"broker.hivemq.com"
// #define MQTT_BROKER_IP IPAddress(18, 196, 225, 29) //"broker.hivemq.com"
#define MQTT_PORT 1883
#define MQTT_TOPIC WIFI_HOSTNAME                           // use WiFi hostname as basic mqtt topic
#define MQTT_OUTPUTINTERVALL_SEC 15                        // controls how often resp. how many of the received data is send to a MQTT broker
#define MQTT_KEEP_ALIVE (MQTT_OUTPUTINTERVALL_SEC * 2.5)   // must be valid together with MQTT_OUTPUTINTERVALL_SEC
#define MQTT_RECONNECTION_TIMEOUT_SEC 15                   // timeout after disconnection from MQTT broker untile a reconnection is tried
#define MIN_FREE_HEAPSIZE_FOR_MQTT_BUFFERQUEUE_BYTES 50000 // MQTT messages are buffered for resending after conditional broker disconnection
                                                           // if this free heap size value is undercut - the oldest buffered values are dropped

#define BUZZER_PIN 25
#define QUIT_BUZZER_BUTTON_PIN 26
#define BEEP_INTERVALL_MS 700

#define PREFERENCES_NAMESPACE "FroelingP2" // limited to 15 chars

/*********************
 *      MACROS
 *********************/
#define XSTRINGIFY(a) STRINGIFY(a)
#define STRINGIFY(a) #a

} // namespace radiator

#endif // #ifndef __DH_CONFIG_H__
