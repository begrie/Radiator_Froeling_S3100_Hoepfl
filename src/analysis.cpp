#include "analysis.h"
#include "debug.h"

#include <chrono>

/*********************
 * STATIC DEFINITIONS
 *********************/
std::string radiator::Analysis::heatingStartTime;
std::string radiator::Analysis::heatingEndTime;
uint16_t radiator::Analysis::heatingDurationThisDayMinutes;
uint8_t radiator::Analysis::heatingStartsThisDay;
float radiator::Analysis::minFuellstand;
float radiator::Analysis::pelletConsumptionThisDayPercent;

/*********************************************************************
 * @brief 	sets a changed radiator status (like "Heizen", "Brenner Aus, ..." and analyse it
 * @param 	time string like 2022-11-26; 10:17:25
 * @return 	void
 *********************************************************************/
void radiator::Analysis::setRadiatorStatus(std::string_view time, std::string_view status)
{
  static std::string lastStatus = "Brenner Aus";

  if (lastStatus.find(status) == std::string::npos) // not found -> status was changed
  {
    lastStatus = status;
    if (status.find("Heizen") != std::string::npos)
    {
      heatingStartsThisDay++;
      heatingStartTime = time;
    }
    else if (status.find("Brenner Aus") != std::string::npos)
    {
      heatingEndTime = time;
      tm zeit;
    }
  }
}

/*********************************************************************
 * @brief 	description
 * @param 	void
 * @return 	void
 *********************************************************************/
std::string radiator::Analysis::getTimeDiff(std::string_view firstTime, std::string_view secondTime)
{
  return "";
}

/*********************************************************************
 * @brief 	calculates the seconds from midnight from time string
 * @param 	time string like 2022-11-26; 10:17:25
 * @return 	seconds from midnight
 *          -1 on failure
 *********************************************************************/
int32_t radiator::Analysis::getSecondsFromMidnight(std::string_view time)
{

  return -1; // failure
}
