#ifndef __DH_OUTPUT_H__
#define __DH_OUTPUT_H__

#include "surveillance.h"

// #include <ostream>
#include <fstream>

namespace radiator
{
#define MQTT_OUTPUTINTERVALL_SEC 60   // controls how often resp. how many of the received data is send to a MQTT broker
                                      // (MQTT output not yet implemented)
#define FILE_OUTPUT_INTERVALL_SEC 10  // controls how often resp. how many of the received data is saved to a logfile
                                      // -> the radiator device sends every second one values data set
                                      //    and the filtering is made by dropping (standard) or averaging (not yet implemented)
                                      //    the values from the previous time series
                                      // -> one values data set needs ca. 200 bytes space in a csv file
                                      //    -> with 60 sec FILE_OUTPUT_INTERVALL_SEC ca. 280kB per day
                                      //       -> ca. 100MB per year (needs SD card output)
                                      //    -> with flash filesystem like littlefs ca. 1,4 to 2 MB available
                                      //       3,8kB to 5,5kB per day allowed to store one years data -> only ca. 1 value per hour
                                      //       -> with 5min(300sec) intervall -> storage for 24 to 34 days
#define WRITE_TO_FILE_INTERVAL_SEC 30 // controls how often the streambuffer of the logfile
                                      // is written to file and how many data can get lost ...
                                      // the greater the value the lower the stress of the flash or SD card
                                      // 30 for testing; later 900 = 15 min for 1 values item per minute
#define DELIMITER_FOR_CSV_FILE "; "

#define BUZZER_PIN 25
#define QUIT_BUZZER_BUTTON_PIN 26
#define BEEP_INTERVALL_MS 700

  class OutputHandler : public radiator::SurveillanceHandler
  {
  protected:
    enum struct FilterMethod_t
    {
      DROP,
      AVERAGE,
      ONCHANGE
    };

    bool toConsole = true;
    bool toMQTT = false;
    bool toFile = false;
    std::string outputPath;     // base path with mountpoint of targeted filesystem -> without filename (e.g. /littlefs/P2_logfiles)
    std::string outputFilename; // one file per day -> is derived from date, template yyyy-mm-dd.log (e.g. /2022-09-01.log)

    uint16_t MQTTOutputIntervallSec = MQTT_OUTPUTINTERVALL_SEC;
    uint16_t fileOutputIntervallSec = FILE_OUTPUT_INTERVALL_SEC;

    typedef std::tuple<ulong, std::string, std::list<VALUE_DATA>> ValuesWithTime_t;
    const std::list<VALUE_DATA> emptyValuesPlaceholder;
    ValuesWithTime_t valuesAtTime = std::make_tuple(0, "0000-00-00, 00:00:00", emptyValuesPlaceholder);
    std::deque<ValuesWithTime_t> valuesTimeSeries;

    std::ofstream outFileStream;
    std::basic_ostream<char> *ostream;

    std::string formatValueData(Surveillance &surveillance, std::list<VALUE_DATA> values);

    std::string formatValueDataHeaderForCSV(Surveillance &surveillance);
    std::string formatValueDataForCSV(std::string time, std::list<VALUE_DATA> values);

    void handleValuesTimeSeries(std::string time);
    void handleValuesTimeSeries(std::list<VALUE_DATA> values);
    ValuesWithTime_t getLastValuesAtTime(FilterMethod_t filterMethod = FilterMethod_t::DROP, uint16_t intervallSec = 0);

    void outputToConsole(std::string output);

    void handleValuesFileOutput(Surveillance &surveillance);
    std::string deriveFilename(std::string stringWithDate);
    bool handleFiles(std::string filename);
    void outputToFile(std::string output, ulong writeToFileIntervalSec = WRITE_TO_FILE_INTERVAL_SEC); // writeToFileIntervalSec controls how often the streambuffer is written to file and how many data can get lost ...

    void outputToMQTT();


  public:
    void outputErrorToBuzzer();
    OutputHandler(std::string pathnameOnly, bool toConsole, bool toMQTT);
    // OutputHandler(std::string filename);
    virtual ~OutputHandler();

    virtual void handleTime(radiator::Surveillance &surveillance,
                            uint8_t dow,
                            uint16_t year, uint8_t month, uint8_t day,
                            uint8_t hour, uint8_t minute, uint8_t second);

    virtual void handleMeasurement(Surveillance &surveillance,
                                   std::list<VALUE_DATA> values);

    virtual void handleError(Surveillance &surveillance,
                             uint16_t year, uint8_t month, uint8_t day,
                             uint8_t hour, uint8_t minute, uint8_t second,
                             std::string description);
  };
}

#endif
