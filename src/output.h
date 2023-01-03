#ifndef __DH_OUTPUT_H__
#define __DH_OUTPUT_H__

#include <fstream>
#include <sstream>

#include "config.h"
#include "network.h"
#include "surveillance.h"


namespace radiator
{
  class OutputHandler : public radiator::SurveillanceHandler
  {
  public:
    OutputHandler(std::string_view pathnameOnly, const bool _toConsole, const bool _toMQTT);
    virtual ~OutputHandler();

    virtual void handleTime(radiator::Surveillance &surveillance,
                            uint8_t dow,
                            uint16_t year, uint8_t month, uint8_t day,
                            uint8_t hour, uint8_t minute, uint8_t second);

    virtual void handleMeasurement(Surveillance &surveillance, std::list<VALUE_DATA> values);

    virtual void handleError(Surveillance &surveillance,
                             uint16_t year, uint8_t month, uint8_t day,
                             uint8_t hour, uint8_t minute, uint8_t second,
                             std::string description);

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

  public:
    typedef std::tuple<time_t, std::string, std::list<VALUE_DATA>> ValuesWithTime_t;

  protected:
    const std::list<VALUE_DATA> emptyValuesPlaceholder;
    ValuesWithTime_t valuesAtTime = std::make_tuple(0, "0000-00-00, 00:00:00", emptyValuesPlaceholder);
    // std::deque<ValuesWithTime_t> valuesTimeSeries;  // needed for buffering and averaging measured values

    std::string bufStr;              // member var instead to local vars to avoid heap fragmentation
    std::ostringstream outStrStream; // for output functions instead to local vars to avoid heap fragmentation
    std::ofstream outFileStream;

    std::string formatValueData(Surveillance &surveillance, const std::list<VALUE_DATA> &values);
    std::string formatValueDataAsJSON(std::string_view timeStringForValues, const std::list<VALUE_DATA> &values);

    std::string formatValueDataHeaderForCSV(Surveillance &surveillance);
    std::string formatValueDataForCSV(std::string_view time, const std::list<VALUE_DATA> &values);

    void handleValuesTimeSeries(std::string_view timeStr);
    void handleValuesTimeSeries(const std::list<VALUE_DATA> &values);
    ValuesWithTime_t getLastValuesAtTime(const FilterMethod_t filterMethod = FilterMethod_t::DROP, const uint16_t intervallSec = 0);

    void outputToConsole(std::string_view output);

    void handleValuesFileOutput(Surveillance &surveillance);
    std::string deriveFilename(const std::string &stringWithDate);
    bool handleFiles(std::string_view filename);
    void outputToFile(std::string_view output, const ulong writeToFileIntervalSec = WRITE_TO_FILE_INTERVAL_SEC); // writeToFileIntervalSec controls how often the streambuffer is written to file and how many data can get lost ...

    void handleValuesMQTTOutput();
    void outputToMQTT(std::string_view output, std::string_view subtopic = "");

    bool checkForRadiatorIsBurning(const std::list<VALUE_DATA> &values);
    bool checkForLimit(const std::list<VALUE_DATA> &values, std::string_view parameterName, const int limit, const bool greaterThan = true);

    void setSystemTimeFromRadiatorData(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second);

    bool radiatorIsBurning = false;
    std::string heatingStartTime;
    std::string heatingEndTime;

    void outputErrorToBuzzer(const uint16_t _beepIntervallMs = BEEP_INTERVALL_RADIATOR_ERROR_MS, const int _timeoutSec = 0);
  };
}

#endif
