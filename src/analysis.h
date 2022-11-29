#ifndef __DH_ANALYSIS_H__
#define __DH_ANALYSIS_H__

#include "config.h"
#include "network.h"

#include "surveillance.h"

namespace radiator
{
  class Analysis
  {
  public:
    static void analyseValues(std::string_view time, const std::list<VALUE_DATA> &values);

  protected:
    static void setRadiatorStatus(std::string_view time, std::string_view status);
    static void setFuellstand(std::string_view time, std::string_view fuellstand);

    static std::string getTimeDiff(std::string_view firstTime, std::string_view secondTime);
    static int32_t getSecondsFromMidnight(std::string_view time);

    static std::string heatingStartTime;
    static std::string heatingEndTime;
    static uint16_t heatingDurationThisDayMinutes;
    static uint8_t heatingStartsThisDay;

    static float minFuellstand;
    static float pelletConsumptionThisDayPercent;
  };
}

#endif //#ifndef __DH_ANALYSIS_H__