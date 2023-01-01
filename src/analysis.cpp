#include "analysis.h"
#include "debug.h"

// #include <chrono>

namespace radiator
{
  /*********************
   * STATIC DEFINITIONS
   *********************/
  radiator::OutputHandler::ValuesWithTime_t *Analysis::ptrValuesAtTime = NULL;
  time_t *Analysis::ptrValueTimet = NULL;
  std::string *Analysis::ptrValueTimeStr;
  std::list<VALUE_DATA> *Analysis::ptrValues;
  std::string Analysis::actualDate;
  time_t Analysis::heatingStartTime = 0;
  time_t Analysis::heatingEndTime = 0;
  uint16_t Analysis::heatingDurationThisDayMinutes = 0;
  uint8_t Analysis::heatingStartsThisDay = 0;
  float Analysis::startFuellstand = 0.0;
  float Analysis::minFuellstand = 0.0;
  float Analysis::refillFuellstandThisDay = 0.0;
  float Analysis::pelletConsumptionThisDayPercent = 0.0;
  std::string Analysis::bufStr;
  std::deque<std::string> Analysis::messages;

  /*********************************************************************
   * @brief 	analyse valuse from one timestamp
   *          concerning heating cycles and duration
   *          and pellets consumption
   * @param 	values tuple with time as time_t and timestring
   * @return 	void
   *********************************************************************/
  void Analysis::analyseValues(radiator::OutputHandler::ValuesWithTime_t &valuesAtTime)
  {
    setValuesAtTime(valuesAtTime);

    if (checkForNewDay())
    {
      if (!actualDate.empty()) // not after startup
      {
        messages.push_back(analyseHeatingCyclesLastDayAndReset());
        messages.push_back(analyseFuellstandLastDayAndReset());
        handleMessages();
      }
      setNewDay();
    }

    traceFuellstand();
    bufStr = checkRadiatorStatusForHeatingCycle();

    if (!bufStr.empty())
    {
      messages.push_back(bufStr);
      handleMessages();
    }
  }

  /*********************************************************************
   * @brief 	handling of messages in deque
   *          - send to MQTT
   *          - limit max. number of messages
   * @param 	void
   * @return 	void
   *********************************************************************/
  void Analysis::handleMessages()
  {
    while (messages.size() > 30)
    {
      messages.pop_front();
    }

    bufStr = "";
    for (auto el : messages)
    {
      bufStr += el;
    }

    NetworkHandler::publishToMQTT(bufStr, MQTT_SUBTOPIC_ANAYLYSIS);
  }

  /*********************************************************************
   * @brief 	set member vars to value data for analysis
   * @param 	values for one timestamp
   * @return 	void
   *********************************************************************/
  void Analysis::setValuesAtTime(radiator::OutputHandler::ValuesWithTime_t &valuesAtTime)
  {
    ptrValuesAtTime = &valuesAtTime;
    ptrValueTimet = &std::get<0>(valuesAtTime);
    ptrValueTimeStr = &std::get<1>(valuesAtTime);
    ptrValues = &std::get<2>(valuesAtTime);
  }

  /*********************************************************************
   * @brief 	get element with parameterName from list with VALUE_DATA
   * @param 	values list for one timestamp
   * @param 	parameter name to find
   * @return 	element with data or empty element if parameterName not found
   *********************************************************************/
  VALUE_DATA Analysis::getElementWithParameterName(std::string_view parameterName)
  {
    for (auto el : *ptrValues)
    {
      if (el.name.find(parameterName) != std::string::npos) // find instead to == due to some possible leading or ending spaces in el.name
      {
        RADIATOR_LOG_DEBUG(millis() << " ms: parameterName=" << parameterName << ", el.name = " << el.name << std::endl;)
        return el;
      }
    }
    RADIATOR_LOG_ERROR(millis() << " ms: getElementWithParameterName(): NO PARAMETER    " << parameterName << "     FOUND" << std::endl;)

    VALUE_DATA emptyElement{};
    return emptyElement;
  }

  /*********************************************************************
   * @brief 	get element with index from list with VALUE_DATA
   * @param 	values list for one timestamp
   * @param 	index to find
   * @return 	element with data or empty element if index not found
   *********************************************************************/
  VALUE_DATA Analysis::getElementWithIndex(uint16_t indexToFind)
  {
    for (auto it = ptrValues->begin(); it != ptrValues->end(); it++)
    {
      if (it->index == indexToFind)
      {
        RADIATOR_LOG_DEBUG(millis() << " ms: index=" << indexToFind << ", it->name = " << it->name << std::endl;)
        return *it;
      }
    }

    // for (auto el : *ptrValues)
    // {
    //   if (el.index == indexToFind)
    //   {
    //     RADIATOR_LOG_DEBUG(millis() << " ms: index=" << indexToFind << ", el.name = " << el.name << std::endl;)
    //     return el;
    //   }
    // }
    RADIATOR_LOG_ERROR(millis() << " ms: getElementWithIndex(): NO INDEX    " << indexToFind << "     FOUND" << std::endl;)

    VALUE_DATA emptyElement{};
    return emptyElement;
  }

  /*********************************************************************
   * @brief 	get element with specified value from value data list
   *          -> e.g. check for "Brenner Aus" and return element ...
   * @param 	values for one timestamp
   * @param 	string to compare as content of .value
   * @return 	found: returns element
   *          NOT found: returns empty element
   *********************************************************************/
  VALUE_DATA Analysis::getElementWithValue(std::string_view valueToFind)
  {
    for (auto el : *ptrValues)
    {
      if (el.value.find(valueToFind) != std::string::npos) // find instead to == due to some possible leading or ending spaces in el.name
        return el;
    }

    VALUE_DATA emptyElement{};
    return emptyElement;
  }

  /*********************************************************************
   * @brief 	checks one parameter from value data list for exceeding a limit
   * @param 	values from one timestamp
   * @param 	name of parameter to check
   * @param 	int limit to check against
   * @param 	true (standard): value greater than limit?
   *          false: value less than limit?
   * @return 	false: limit NOT exceeded
   *          true : limit exceeded
   *********************************************************************/
  bool Analysis::checkForLimit(std::string_view parameterName, const int limit, const bool greaterThan)
  {
    RADIATOR_LOG_DEBUG(millis() << " ms: checkForLimit -> parameterName=" << parameterName << ", limit= " << limit << std::endl;)

    auto el = getElementWithParameterName(parameterName);
    RADIATOR_LOG_DEBUG(millis() << " ms: parameterName=" << parameterName << ", el.name = " << el.name << std::endl;)

    if (el.name.empty()) // parameterName not found
      return false;      //-> therefore the limit is not exceeded ...

    // int valueAsInt = std::stoi(el.value);  //stoi can throw an expection -> so better to use atoi
    int valueAsInt = atoi(el.value.c_str());
    RADIATOR_LOG_DEBUG(millis() << " ms: valueAsInt= " << valueAsInt << std::endl;)

    if ((greaterThan && valueAsInt > limit) || (!greaterThan && valueAsInt < limit))
    {
      RADIATOR_LOG_DEBUG(millis() << " ms: Limit EXCEEDED" << std::endl;)
      return true;
    }
    else // parameter found, limit not exceeded
    {
      RADIATOR_LOG_DEBUG(millis() << " ms: Limit NOT exceeded " << std::endl;)
      return false;
    }
  }

  /*********************************************************************
   * @brief 	find value (as string) from value data list
   *          -> e.g. check for "Brenner Aus" ... as value NOT as parameterName
   * @param 	values from one timestamp
   * @param 	string to compare as content of .value
   * @return 	true : found
   *          false: NOT found
   *********************************************************************/
  bool Analysis::findValue(std::string_view valueToFind)
  {
    for (auto el : *ptrValues)
    {
      if (el.value.find(valueToFind) != std::string::npos) // find instead to == due to some possible leading or ending spaces in el.name
        return true;
    }

    return false;
  }

  /*********************************************************************
   * @brief 	check for change of day
   * @param 	void
   * @return 	true: day change
   *          false: same day as meber var actualDate
   *********************************************************************/
  bool Analysis::checkForNewDay()
  {
    if (actualDate != ptrValueTimeStr->substr(0, 10)) // compares with date like 2022-12-19 from valueTimeStr like 2022-12-19;13:58:50
      return true;

    return false;
  }

  /*********************************************************************
   * @brief 	description
   * @param 	void
   * @return 	void
   *********************************************************************/
  void Analysis::setNewDay()
  {
    actualDate = ptrValueTimeStr->substr(0, 10);
  }

  /*********************************************************************
   * @brief 	checks radiator status (like "Heizen", "Brenner Aus, ...") and traces changes
   * @param 	time string like 2022-11-26; 10:17:25
   * @return 	message with info abot heating cycle
   *          empty string if status was NOT changed to last call
   *********************************************************************/
  std::string Analysis::checkRadiatorStatusForHeatingCycle()
  {
    static std::string lastStatus = "";

    auto status = getElementWithIndex(1).value;

    if (lastStatus.find(status) == std::string::npos) // not found -> status was changed
    {
      lastStatus = status;
      if (status.find("Heizen") != std::string::npos)
      {
        heatingStartTime = time(NULL);
        heatingStartsThisDay++;

        return (*ptrValueTimeStr + ": Start Heizen (Fuellst.= " + getFuellstandAsString() + ") \n");
      }
      else if (status.find("Brenner Aus") != std::string::npos)
      {
        heatingEndTime = time(NULL);
        heatingDurationThisDayMinutes += (heatingEndTime - heatingStartTime) / 60;

        return (*ptrValueTimeStr + ": Brenner Aus (Fuellst.= " + getFuellstandAsString() + "), letzte Heizdauer= " + std::to_string((heatingEndTime - heatingStartTime) / 60) + " Minuten) \n");
      }
    }

    return "";
  }

  /*********************************************************************
   * @brief 	analyse heating cycles for the last day
   *          to calculate runtime
   *          and resets all relevant values for the next day
   * @param 	void
   * @return 	string with message about heating cycles
   *********************************************************************/
  std::string Analysis::analyseHeatingCyclesLastDayAndReset()
  {
    // generate message
    bufStr = actualDate + ": Tages-Heizdauer= " + std::to_string(heatingDurationThisDayMinutes) + " Minuten, Brennerstarts= " + std::to_string(heatingStartsThisDay) + " \n";

    // reset values
    heatingDurationThisDayMinutes = 0;
    heatingStartsThisDay = 0;

    return bufStr;
  }

  /*********************************************************************
   * @brief 	returns Fuellstand from values list
   * @param 	void
   * @return 	Fuellstand as float or -1.0 on error
   *********************************************************************/
  float Analysis::getFuellstand()
  {
    auto el = getElementWithParameterName("Fuellst.:");

    if (el.name.empty())
      return -1.0;

    auto fuellstand = strtof(el.value.c_str(), NULL); // in case of error strtof() returns 0.0  !!

    if (fuellstand != 0.0)
      return fuellstand;
    else
      return -1.0;
  }

  /*********************************************************************
   * @brief 	returns Fuellstand from values list as string
   * @param 	void
   * @return 	Fuellstand as string or "ERROR" on error
   *********************************************************************/
  std::string Analysis::getFuellstandAsString()
  {
    auto el = getElementWithParameterName("Fuellst.:");

    if (el.name.empty())
      return "ERROR";

    return el.value;
  }

  /*********************************************************************
   * @brief 	traces min/max values and refill of Fuellstand
   * @param 	values from one timestamp
   * @return 	void
   *********************************************************************/
  void Analysis::traceFuellstand()
  {
    auto fuellstand = getFuellstand();

    if (fuellstand < minFuellstand)
    {
      minFuellstand = fuellstand;
      pelletConsumptionThisDayPercent = startFuellstand - minFuellstand;
    }
    else if (fuellstand > minFuellstand) // refill was made
    {
      refillFuellstandThisDay += fuellstand - minFuellstand; // += for more than 1 refills per day
      minFuellstand = startFuellstand = fuellstand;
    }
  }

  /*********************************************************************
   * @brief 	analyse traced fuellstand values for the last day
   *          to calculate pellets consumption
   *          and resets all relevant values for the next day
   * @param 	void
   * @return 	string with message about pellets consumption
   *********************************************************************/
  std::string Analysis::analyseFuellstandLastDayAndReset()
  {
    pelletConsumptionThisDayPercent += refillFuellstandThisDay - minFuellstand;

    // generate message
    bufStr = actualDate + ": Tages-Pelletsverbrauch= " + std::to_string(pelletConsumptionThisDayPercent) + "% \n";

    // reset values
    pelletConsumptionThisDayPercent = 0.0;
    refillFuellstandThisDay = 0.0;
    minFuellstand = startFuellstand = getFuellstand();

    return bufStr;
  }

} // namespace radiator