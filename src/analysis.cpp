#include "analysis.h"
#include "debug.h"

// #include <chrono>

namespace radiator
{
  /*********************
   * STATIC DEFINITIONS
   *********************/
  Preferences Analysis::preferences;
  radiator::OutputHandler::ValuesWithTime_t *Analysis::ptrValuesAtTime = NULL;
  time_t *Analysis::ptrValueTimet = NULL;
  std::string *Analysis::ptrValueTimeStr;
  std::list<VALUE_DATA> *Analysis::ptrValues;
  std::string Analysis::actualDate;
  std::string Analysis::lastRadiatorStatus = "";
  time_t Analysis::heatingStartTime = 0;
  time_t Analysis::heatingEndTime = 0;
  uint16_t Analysis::heatingDurationThisDayMinutes = 0;
  uint16_t Analysis::heatingPauseMinutes = 0;
  uint8_t Analysis::heatingStartsThisDay = 0;
  float Analysis::startFuellstand = 100.0;
  float Analysis::minFuellstand = 100.0;
  float Analysis::refillFuellstandThisDay = 0.0;
  float Analysis::pelletConsumptionThisDayPercent = 0.0;
  std::string Analysis::bufStr;
  std::deque<std::string> Analysis::messages;

  /*********************************************************************
   * @brief 	init after startup of ESP
   *          -> should only be called after setting/synchronization of system time to radiator time
   *          -> loads last infos for analysis from preferences
   * @param 	void
   * @return 	void
   *********************************************************************/
  void Analysis::init()
  {
    static bool initWasDone = false;

    bufStr.reserve(3000);

    if (time(NULL) < 1640991600) // check against time_t of 01.01.2022
    {
      RADIATOR_LOG_ERROR(getMillisAndTime() << "Cannot load data for analysis from preferences, because ESP time is not synchronized to radiator time");
      return;
    }

    if (initWasDone)
      return;

    preferences.begin(PREFERENCES_NAMESPACE);

    heatingStartTime = preferences.getLong("heatingStartTim", 0);
    heatingEndTime = preferences.getLong("heatingEndTime", 0);
    heatingDurationThisDayMinutes = preferences.getUShort("heatingDuration", 0);
    heatingPauseMinutes = preferences.getUShort("heatingPauseMin", 0);
    heatingStartsThisDay = preferences.getUChar("heatingStartsTh", 0);

    startFuellstand = preferences.getFloat("startFuellstand", 100);
    minFuellstand = preferences.getFloat("minFuellstand", 100);
    refillFuellstandThisDay = preferences.getFloat("refillFuellstan", 0);
    pelletConsumptionThisDayPercent = preferences.getFloat("pelletConsumpti", 0);

    bufStr = (preferences.getString("analyseMessages", "")).c_str();

    preferences.end();

    if (!bufStr.empty())
    {
      size_t posStart, posEnd = 0;
      while (posEnd = bufStr.find("\n", posStart) != std::string::npos)
      {
        Serial.printf("AAAA: %s \n", bufStr.substr(posStart, posEnd - posStart + 1).c_str());
        messages.push_back(bufStr.substr(posStart, posEnd - posStart + 1));
        posStart = posEnd + 1;
      }
    }

    initWasDone = true;
  }

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
    while (messages.size() > MAX_ANALYSE_MESSAGES)
    {
      messages.pop_front();
    }

    bufStr = "";
    for (auto el : messages)
    {
      bufStr += el;
    }

    NetworkHandler::publishToMQTT(bufStr, MQTT_SUBTOPIC_ANAYLYSIS);

    preferences.begin(PREFERENCES_NAMESPACE);
    preferences.putString("analyseMessages", bufStr.c_str());
    preferences.end();
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
        RADIATOR_LOG_DEBUG(getMillisAndTime() << "parameterName=" << parameterName << ", el.name = " << el.name << std::endl;)
        return el;
      }
    }

    RADIATOR_LOG_ERROR(getMillisAndTime() << "getElementWithParameterName(): NO PARAMETER    " << parameterName << "     FOUND" << std::endl;)
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
        RADIATOR_LOG_DEBUG(getMillisAndTime() << "index=" << indexToFind << ", it->name = " << it->name << std::endl;)
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

    RADIATOR_LOG_ERROR(getMillisAndTime() << "getElementWithIndex(): NO INDEX    " << indexToFind << "     FOUND" << std::endl;)
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
    RADIATOR_LOG_DEBUG(getMillisAndTime() << "checkForLimit -> parameterName=" << parameterName << ", limit= " << limit << std::endl;)

    auto el = getElementWithParameterName(parameterName);
    RADIATOR_LOG_DEBUG(getMillisAndTime() << "parameterName=" << parameterName << ", el.name = " << el.name << std::endl;)

    if (el.name.empty()) // parameterName not found
      return false;      //-> therefore the limit is not exceeded ...

    // int valueAsInt = std::stoi(el.value);  //stoi can throw an expection -> so better to use atoi
    int valueAsInt = atoi(el.value.c_str());
    RADIATOR_LOG_DEBUG(getMillisAndTime() << "valueAsInt= " << valueAsInt << std::endl;)

    if ((greaterThan && valueAsInt > limit) || (!greaterThan && valueAsInt < limit))
    {
      RADIATOR_LOG_DEBUG(getMillisAndTime() << "Limit EXCEEDED" << std::endl;)
      return true;
    }
    else // parameter found, limit not exceeded
    {
      RADIATOR_LOG_DEBUG(getMillisAndTime() << "Limit NOT exceeded " << std::endl;)
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
    auto status = getElementWithIndex(1).value;

    if (lastRadiatorStatus.find(status) == std::string::npos // not found -> status was changed
        && status.find("Abreinigen") == std::string::npos)   // ignorieren von "Abreinigen" -> erfolgt innerhalb eines Heizzyklus - d.h. "Heizen" -"Abreinigen" - "Heizen" - ... - "Brenner Aus"
    {
      // bool abreinigen;
      // if (lastRadiatorStatus.find("Abreinigen") == std::string::npos) //"Abreinigen" erfolgt innerhalb eines Heizzyklus - d.h. "Heizen" -"Abreinigen" - "Heizen" - ... - "Brenner Aus"
      //   abreinigen = false;
      // else
      //   abreinigen = true;

      lastRadiatorStatus = status;
      // if (status.find("Heizen") != std::string::npos && lastRadiatorStatus.find("Abreinigen") == std::string::npos) //"Abreinigen" erfolgt innerhalb eines Heizzyklus - d.h. "Heizen" -"Abreinigen" - "Heizen" - ... - "Brenner Aus"
      if (status.find("Heizen") != std::string::npos)
      {
        heatingStartTime = time(NULL);
        heatingPauseMinutes = (heatingEndTime - heatingStartTime) / 60;
        heatingStartsThisDay++;

        preferences.begin(PREFERENCES_NAMESPACE);
        preferences.putString("lastRadiatorSta", lastRadiatorStatus.c_str());
        preferences.putLong("heatingStartTim", heatingStartTime);
        preferences.putUShort("heatingPauseMin", heatingPauseMinutes);
        preferences.putUChar("heatingStartsTh", heatingStartsThisDay);
        preferences.end();

        return (*ptrValueTimeStr + ": Start Heizen (Fuellst.= " + getFuellstandAsString() + "), letzte Heizpause= " + std::to_string(heatingPauseMinutes) + " Minuten" DELIMITER_FOR_CSV_FILE "\n");
      }
      else if (status.find("Brenner Aus") != std::string::npos)
      {
        heatingEndTime = time(NULL);
        heatingDurationThisDayMinutes += (heatingEndTime - heatingStartTime) / 60;

        preferences.begin(PREFERENCES_NAMESPACE);
        preferences.putString("lastRadiatorSta", lastRadiatorStatus.c_str());
        preferences.putLong("heatingEndTime", heatingEndTime);
        preferences.putUShort("heatingDuration", heatingDurationThisDayMinutes);
        preferences.end();

        return (*ptrValueTimeStr + ": Brenner Aus (Fuellst.= " + getFuellstandAsString() + "), letzte Heizdauer= " + std::to_string((heatingEndTime - heatingStartTime) / 60) + " Minuten)" DELIMITER_FOR_CSV_FILE "\n");
      }
    }

    return "";
  }

  /*********************************************************************
   * @brief 	analyse heating cycles for the last day
   *          -> to calculate runtime
   *          -> and resets all relevant values for the next day
   * @param 	void
   * @return 	string with message about heating cycles
   *********************************************************************/
  std::string Analysis::analyseHeatingCyclesLastDayAndReset()
  {
    // generate message
    bufStr = actualDate + ": Tages-Heizdauer= " + std::to_string(heatingDurationThisDayMinutes) + " Minuten, Brennerstarts= " + std::to_string(heatingStartsThisDay) + DELIMITER_FOR_CSV_FILE "\n";

    // reset values
    heatingDurationThisDayMinutes = 0;
    heatingStartsThisDay = 0;

    // save to NVS
    preferences.begin(PREFERENCES_NAMESPACE);
    preferences.putUShort("heatingDuration", heatingDurationThisDayMinutes);
    preferences.putUChar("heatingStartsTh", heatingStartsThisDay);
    preferences.end();

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

    static time_t timeLastChange = 0;

    if (fuellstand == minFuellstand)
    {
      if (time(NULL) - timeLastChange > 120) // save only after 2 minutes with same values to take care for the flash memory write cycles
      {
        timeLastChange = LONG_MAX;
        preferences.begin(PREFERENCES_NAMESPACE);
        preferences.putFloat("startFuellstand", startFuellstand);
        preferences.putFloat("minFuellstand", minFuellstand);
        preferences.putFloat("refillFuellstan", refillFuellstandThisDay);
        preferences.end();
      }
      return;
    }

    if (fuellstand < minFuellstand)
    {
      minFuellstand = fuellstand;
      // pelletConsumptionThisDayPercent = startFuellstand - minFuellstand;
      timeLastChange = time(NULL);
    }
    else if (fuellstand > minFuellstand) // refill was made
    {
      refillFuellstandThisDay += fuellstand - minFuellstand; // += for more than 1 refills per day
      minFuellstand = startFuellstand = fuellstand;
      timeLastChange = time(NULL);
    }
  }

  /*********************************************************************
   * @brief 	analyse traced fuellstand values for the last day
   *          -> to calculate pellets consumption
   *          -> and resets all relevant values for the next day
   * @param 	void
   * @return 	string with message about pellets consumption
   *********************************************************************/
  std::string Analysis::analyseFuellstandLastDayAndReset()
  {
    pelletConsumptionThisDayPercent = refillFuellstandThisDay + (startFuellstand - getFuellstand());
    // pelletConsumptionThisDayPercent += refillFuellstandThisDay + (startFuellstand - minFuellstand);

    // generate message
    bufStr = actualDate + ": Tages-Pelletsverbrauch= " + std::to_string(pelletConsumptionThisDayPercent) + "%" DELIMITER_FOR_CSV_FILE "\n";

    // reset values
    // pelletConsumptionThisDayPercent = 0.0;
    refillFuellstandThisDay = 0.0;
    startFuellstand = getFuellstand();
    // minFuellstand = startFuellstand = getFuellstand();

    // save to NVS
    preferences.begin(PREFERENCES_NAMESPACE);
    preferences.putFloat("startFuellstand", startFuellstand);
    // preferences.putFloat("minFuellstand", minFuellstand);
    preferences.putFloat("refillFuellstan", refillFuellstandThisDay);
    // preferences.putFloat("pelletConsumpti", pelletConsumptionThisDayPercent);
    preferences.end();

    return bufStr;
  }

} // namespace radiator