#ifndef __DH_ANALYSIS_H__
#define __DH_ANALYSIS_H__

#include "config.h"
#include "network.h"
#include "surveillance.h"
#include "output.h"

#include <Preferences.h>

namespace radiator
{
  class Analysis
  {
  public:
    static void init();
    static void analyseValues(radiator::OutputHandler::ValuesWithTime_t &valuesAtTime);

  protected:
    static Preferences preferences;

    static void setValuesAtTime(radiator::OutputHandler::ValuesWithTime_t &valuesAtTime);
    static radiator::OutputHandler::ValuesWithTime_t *ptrValuesAtTime;
    static time_t *ptrValueTimet;
    static std::string *ptrValueTimeStr;
    static std::list<VALUE_DATA> *ptrValues;

    static VALUE_DATA getElementWithParameterName(std::string_view parameterName);
    static VALUE_DATA getElementWithValue(std::string_view valueToFind);
    static VALUE_DATA getElementWithIndex(uint16_t indexToFind);

    static bool checkForLimit(std::string_view parameterName, const int limit, const bool greaterThan);
    static bool findValue(std::string_view valueToFind);
    static bool checkForNewDay();
    static void setNewDay();
    static std::string actualDate;

    //    static void saveKeyValueToPreferences();

    static std::string checkRadiatorStatusForHeatingCycle();
    static std::string analyseHeatingCyclesLastDayAndReset();

    static std::string lastRadiatorStatus;
    static time_t heatingStartTime;
    static time_t heatingEndTime;
    static uint16_t heatingDurationThisDayMinutes;
    static uint16_t heatingPauseMinutes;
    static uint8_t heatingStartsThisDay;

    static float getFuellstand();
    static std::string getFuellstandAsString();
    static void traceFuellstand();
    static std::string analyseFuellstandLastDayAndReset();

    static float startFuellstand;
    static float minFuellstand;
    static float refillFuellstandThisDay;
    static float pelletConsumptionThisDayPercent;

    static void handleMessages();
    static std::string bufStr;
    static std::deque<std::string> messages;
  };
}

#endif // #ifndef __DH_ANALYSIS_H__