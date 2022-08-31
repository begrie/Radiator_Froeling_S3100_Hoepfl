#include <Arduino.h>
#include <LittleFS.h>

#include "debug.h"
#include "device.h"
#include "surveillance.h"
#include "output.h"

#include <getopt.h>
#include <unistd.h> //for getopt -> only?

#include <cstdlib>
#include <iostream>
#include <iomanip>

#if HAVE_CONFIG_H
#include "config.h"
#endif

#define D_DEBUG_LEVEL 3
#define T_TIMEOUT_BETWEEN_TRANSFERS_S 5
#define O_OUTPUT_STREAM "-" //"/20220830_Testausgabe.dat"
#define S_SERIAL_TO_RADIATOR "Serial2"
// #define SERIAL_TO_RADIATOR_RX 16 // ESP32-standard: 16
// #define SERIAL_TO_RADIATOR_TX 17 // ESP32-standard: 17

void versionInfo(std::ostream &ostream)
{
  //  ostream << PACKAGE_NAME << " V" << PACKAGE_VERSION << std::endl;
  //  ostream << "Report bugs to " << PACKAGE_BUGREPORT << std::endl;
  ostream << std::endl;
}

void usage(std::ostream &ostream, std::string argv0)
{
  ostream << "Usage: " << argv0 << " [-V] [-h] [-D[level]] [-T <timeout in s>] [-o <output>] <serial port name>" << std::endl;
  ostream << std::endl;
  ostream << "   -V                 Print version info and exit." << std::endl;
  ostream << "   -h                 Print this usage info and exit." << std::endl;
  ostream << std::endl;
  ostream << "   -D[level]          Increase log level or set to the given level." << std::endl;
  ostream << std::endl;
  ostream << "                      Available levels:" << std::endl;
  ostream << std::endl;
  ostream << "                        0  - No logging" << std::endl;
  ostream << "                        1  - Errors" << std::endl;
  ostream << "                        2  - Warnings" << std::endl;
  ostream << "                        3  - Infos" << std::endl;
  ostream << "                        4  - Debug" << std::endl;
  ostream << "                        5  - Trace" << std::endl;
  ostream << std::endl;
  ostream << "   -T <timeout in s>  Maximum time allowed between transfers until" << std::endl;
  ostream << "                      connection is considered stale." << std::endl;
  ostream << std::endl;
  ostream << "   -o <output>        Where to write the received values to." << std::endl;
  ostream << "                      \"-\" writes to stdout." << std::endl;
  ostream << std::endl;
  ostream << "   <serial port>      Filename of the serial device." << std::endl;
  ostream << std::endl;

  versionInfo(ostream);
}

// int main(int argc, char *argv[])
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
  char devicename[] = S_SERIAL_TO_RADIATOR;
  int timeout = T_TIMEOUT_BETWEEN_TRANSFERS_S * 1000;
  std::string output = O_OUTPUT_STREAM;

  if (!LittleFS.begin(false))
  {
    Serial.println("LittleFS Mount Failed -> Try to format file system for LittleFS");
    if (!LittleFS.begin(true)) // open with format filesystem
    {
      Serial.println("LittleFS Mount Failed AGAIN -> Formatting not possible -> NO LOCAL DATA STORAGE AVAILABLE");
      output = "-";
    }
  }

  if (output == "-")
  {
    std::cout << "Data output ONLY to console (std::cout)" << std::endl;
  }
  else
  {
    std::cout << "Data is stored on LittleFS at " << output
              << " (Available space " << (LittleFS.totalBytes() - LittleFS.usedBytes())
              << " bytes from total " << LittleFS.totalBytes() << " bytes)" << std::endl;
  }

  // debug_level = 1;

  // int timeout = 1500;

  // std::string output = "";
  // for (int c = 0; (c = ::getopt(argc, argv, "+D::T:o:hV")) != -1;)
  // {
  //   switch (c)
  //   {
  //   case 'D':
  //     if (optarg)
  //     {
  //       debug_level = ::atoi(optarg);
  //     }
  //     else
  //     {
  //       debug_level++;
  //     }
  //     break;
  //   case 'T':
  //     timeout = atoi(optarg) * 1000;
  //     if (timeout < 1000)
  //     {
  //       LOG_info << "Min timeout is 1s, adjusted" << std::endl;
  //       timeout = 1000;
  //     }

  //     break;
  //   case 'o':
  //     output = optarg;
  //     break;
  //   case '?':
  //     if (optopt == 'T' || optopt == 'o')
  //       LOG_error << "Option -" << optopt << " requires an argument." << std::endl;
  //     else if (::isprint(optopt))
  //       LOG_error << "Unknown option `-" << (char)optopt << "'." << std::endl;
  //     else
  //       LOG_error << "Unknown option character `\\x" << std::hex << (unsigned int)optopt << std::dec << "'." << std::endl;
  //     ::exit(1);
  //   case 'h':
  //     usage(std::cerr, argv[0]);
  //     ::exit(1);
  //   case 'V':
  //     versionInfo(std::cerr);
  //     ::exit(1);
  //   }
  // }

  // if (argc - optind != 1)
  // {
  //   usage(LOG_error, argv[0]);
  //   ::exit(1);
  // }

  // char *devicename = argv[optind];

  try
  {
    for (;;)
    {
      LOG_trace << "Starting main loop" << std::endl;

      radiator::OutputHandler handler(output);
      radiator::Surveillance surveillance(devicename, timeout, handler);
      surveillance.main_loop();

      LOG_info << "Main loop ended, restarting in 10s" << std::endl;
      sleep(10);
    }
  }
  catch (const char *&error)
  {
    LOG_fatal << "Failed to open serial device, quitting" << std::endl;
  }
}

/*********************************************************************
 * @brief 	Loop function handles only GUI with lvgl handler
 * @param 	void
 * @return 	void
 *********************************************************************/
void loop()
{
}
