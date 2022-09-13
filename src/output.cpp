#include "output.h"

#include "debug.h"

#include <fstream>
// #include <iostream>
#include <iomanip>

#include <regex>

namespace radiator
{
  OutputHandler::OutputHandler(std::string pathnameOnly, bool toConsole, bool toMQTT)
      : outputPath(pathnameOnly), toConsole(toConsole), toMQTT(toMQTT)
  {
    outputPath.empty() ? toFile = false : toFile = true;

    LOG_info << "Output to console " << (toConsole ? "ON" : "OFF") << std::endl;
    LOG_info << "Output to MQTT " << (toMQTT ? "ON" : "OFF") << std::endl;
    LOG_info << "Output to file " << (toFile ? "ON" : "OFF") << std::endl;

    if (toFile)
    {
      if (outputPath.back() != '/')
        outputPath += "/";

      // check for path availability (with fstream only possible by usage of a testfile ...)
      std::ofstream testForDirectory(outputPath + "dirtest.tst");
      if (!testForDirectory)
      {
        LOG_error << "Output path  " << outputPath << "  CAN NOT BE OPENED -> NO VALUES WILL BE SAVED TO FILE" << std::endl;
        // toFile = false;
        // return;
        ::perror("OutputHandler: Unable to open file for output");
        throw("Unable to open file for output");
      }

      LOG_info << "Files are saved to path   " << outputPath << std::endl;
    }
  }

  OutputHandler::~OutputHandler()
  {
  }

  /*********************************************************************
   * @brief 	The radiator sends every second a "M2" command with the actual time
   *          followed by a "M1" command with the values at this time
   *          ->  in this function the "M2" is handled by formatting the radiator time
   *              and pushing it to the next processing steps
   * @param 	instance of Surveillance working class
   * @param   ... date & time ...
   * @return 	void
   *********************************************************************/
  void OutputHandler::handleTime(Surveillance &surveillance,
                                 uint8_t dow,
                                 uint16_t year, uint8_t month, uint8_t day,
                                 uint8_t hour, uint8_t minute, uint8_t second)
  {
    std::ostringstream outStrStream;

    outStrStream << std::dec
                 << std::setw(4) << std::setfill('0') << (int)year << "-"
                 << std::setw(2) << std::setfill('0') << (int)month << "-"
                 << std::setw(2) << std::setfill('0') << (int)day << DELIMITER_FOR_CSV_FILE
                 << std::setw(2) << std::setfill('0') << (int)hour << ":"
                 << std::setw(2) << std::setfill('0') << (int)minute << ":"
                 << std::setw(2) << std::setfill('0') << (int)second;

    static const char *dowString[7] = {"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};

    if (toConsole)
    {
      std::ostringstream enrichedOutput;
      enrichedOutput << "[TIME] "
                     << dowString[dow - 1] << ", "
                     << outStrStream.str() << std::endl;
      outputToConsole(enrichedOutput.str());
    }

    handleValuesTimeSeries(outStrStream.str());
  }

  /*********************************************************************
   * @brief 	handling of the measurement values received from the radiator
   *          with the "M1" command
   * @param 	instance of Surveillance working class
   * @param 	list with values from radiator
   * @return 	void
   *********************************************************************/
  void OutputHandler::handleMeasurement(Surveillance &surveillance,
                                        std::list<VALUE_DATA> values)
  {
    if (toConsole)
      outputToConsole(formatValueData(surveillance, values));

    handleValuesTimeSeries(values);

    if (toFile)
      handleValuesFileOutput(surveillance);
  }

  /*********************************************************************
   * @brief 	handling of error messages received from the radiator
   * @param  	instance of Surveillance working class
   * @param 	... date & time ...
   * @param   error description
   * @return 	void
   *********************************************************************/
  void OutputHandler::handleError(Surveillance &surveillance,
                                  uint16_t year, uint8_t month, uint8_t day,
                                  uint8_t hour, uint8_t minute, uint8_t second,
                                  std::string description)
  {
    std::ostringstream outStrStream;

    outStrStream << "[ERROR] "
                 << std::dec
                 << std::setw(4) << std::setfill('0') << (int)year << "-"
                 << std::setw(2) << std::setfill('0') << (int)month << "-"
                 << std::setw(2) << std::setfill('0') << (int)day << ", "
                 << std::setw(2) << std::setfill('0') << (int)hour << ":"
                 << std::setw(2) << std::setfill('0') << (int)minute << ":"
                 << std::setw(2) << std::setfill('0') << (int)second << ": "
                 << description << std::endl;

    if (toConsole)
      outputToConsole(outStrStream.str());

    outputErrorToBuzzer();

    if (toFile)
    {
      if (handleFiles(deriveFilename(outStrStream.str())))
      {
        outputToFile(outStrStream.str());
      }
      else
        LOG_error << "ERROR saving to File" << std::endl;
    }
  }

  /*********************************************************************
   * @brief 	format received value data to string for output to console or/and file
   * @param  	instance of Surveillance working class
   * @param 	list with values from radiator
   * @return 	formatted string with received values
   *          e.g.:    [VALUE] 00 [] = [Ausgeschaltet ] [S] (6)
   *                   [VALUE] 01 [] = [  Brenner Aus  ] [S] (1)
   *                   [VALUE] 02 [Zustand] = [1] [N] (1)
   *                   [VALUE] 03 [ROST] = [0] [N] (0)
   *                   [VALUE] 04 [Kesseltemp] = [18°] [N] (37)
   *                   [VALUE] 05 [Abgastemp.] = [21°] [N] (21)
   *                   [VALUE] 06 [Abgas. SW ] = [31°] [N] (31)
   *                   [VALUE] 07 [KessStellGr] = [50%] [N] (50)
   *                   [VALUE] 08 [Saugzug  ] = [0%] [N] (0)
   *                   [VALUE] 09 [Zuluftgebl] = [0%] [N] (0)
   *                   [VALUE] 10 [Einschub] = [0%] [N] (0)
   *                   [VALUE] 11 [Fuellst.:] = [99.9%] [N] (20673)
   *                   [VALUE] 12 [Feuerraumt] = [347°] [N] (347)
   *                   [VALUE] 13 [Puffert.ob] = [18°] [N] (37)
   *                   [VALUE] 14 [Puffert.un] = [19°] [N] (38)
   *                   [VALUE] 15 [Puffer Pu.] = [0%] [N] (0)
   *                   [VALUE] 16 [Außentemp] = [17°] [N] (34)
   *                   [VALUE] 17 [Vorlauft.1sw] = [0°] [N] (0)
   *                   [VALUE] 18 [Vorlauft.1] = [20°] [N] (41)
   *                   [VALUE] 19 [Vorlauft.2sw] = [0°] [N] (0)
   *                   [VALUE] 20 [Vorlauft.2] = [20°] [N] (39)
   *                   [VALUE] 21 [KTY6_H2] = [127°] [N] (254)
   *                   [VALUE] 22 [KTY7_H2] = [127°] [N] (254)
   *                   [VALUE] 23 [Brenn.ST] = [10461] [N] (10461)
   *                   [VALUE] 24 [Laufzeit:] = [15878h] [N] (15878)
   *                   [VALUE] 25 [Boardtemp.] = [21°] [N] (21)
   *                   [VALUE] 26 [Die Kesseltemp. soll sein] = [75°] [N] (150)
   *********************************************************************/
  std::string OutputHandler::formatValueData(Surveillance &surveillance,
                                             std::list<VALUE_DATA> values)
  {
    std::ostringstream outStrStream;

    auto parameterNames = surveillance.getParameterNames();
    for (auto iter = values.begin();
         iter != values.end();
         ++iter)
    {
      outStrStream
          //<< "[VALUE] "
          //<< std::dec << std::setw(2) << std::setfill('0') << iter->index << std::dec
          << " [" << iter->name << "] = [" << iter->value << "]";

      switch (parameterNames[iter->index].type)
      {
      case PNT_STRING:
        outStrStream << " [S]";
        break;
      case PNT_VALUE:
        outStrStream << " [N]";
        break;
      default:
        outStrStream << " [O]";
        break;
      }

      outStrStream
          << " (" << (int)iter->rawValue << ")\n";
    }
    outStrStream << std::endl;

    return outStrStream.str();
  }

  /*********************************************************************
   * @brief 	format data for header of a csv file
   * @param  	instance of Surveillance working class
   * @return 	formatted string with parameter names received from radiator -> delimiter is ; (semicolon)
   *          e.g.: Date; Time;  [S];  [S]; Zustand [N]; ROST [N]; Kesseltemp [N]; Abgastemp. [N]; Abgas. SW  [N]; KessStellGr [N]; Saugzug   [N]; Zuluftgebl [N]; Einschub [N]; Fuellst.: [N]; Feuerraumt [N]; Puffert.ob [N]; Puffert.un [N]; Puffer Pu. [N]; Außentemp [N]; Vorlauft.1sw [N]; Vorlauft.1 [N]; Vorlauft.2sw [N]; Vorlauft.2 [N]; KTY6_H2 [N]; KTY7_H2 [N]; Brenn.ST [N]; Laufzeit: [N]; Boardtemp. [N]; Die Kesseltemp. soll sein [N];
   *********************************************************************/
  std::string OutputHandler::formatValueDataHeaderForCSV(Surveillance &surveillance)
  {
    std::ostringstream outStrStream;

    outStrStream << "Date; Time" << DELIMITER_FOR_CSV_FILE;

    auto parameterNames = surveillance.getParameterNames();
    for (auto el : parameterNames)
    {
      outStrStream << el.name; // this is how it should work ...

      // // ...but there is an unwanted CRLF (\r\n) in the parameter "Vorlauft.1sw" (end evtl. others ...?)
      // //-> so lets remove them
      // auto paramName = el.name;
      // LOG_info << "paramName=" << paramName << " ->paramName.find(CR)=" << (paramName.find('\r'))
      //          << ", paramName.find(LF)=" << (paramName.find('\n')) << std::endl;
      // // paramName.replace(paramName.find('\r'), 1, "");  //-> crashes -> needs check for length vs. pos
      // // paramName.replace(paramName.find('\n'), 1, "");  //-> crashes
      // outStrStream << paramName;

      switch (el.type)
      {
      case PNT_STRING:
        outStrStream << " [S]";
        break;
      case PNT_VALUE:
        outStrStream << " [N]";
        break;
      default:
        outStrStream << " [O]";
        break;
      }

      outStrStream << DELIMITER_FOR_CSV_FILE;
    }

    outStrStream << std::endl;

    LOG_info << "formatValueDataHeaderForCSV = " << outStrStream.str() << std::endl;

    return outStrStream.str();
  }

  /*********************************************************************
   * @brief 	format values from radiator for output as CSV file
   * @param 	string with date and time e.g.  22-09-01; 12:21:35
   * @param 	list of radiator values at one time
   * @return 	formatted string with received values
   *          e.g.  22-09-01; 12:21:35; Ausgeschaltet; Brenner Aus; 1; 0; 18°; 21°; 31°; 50%; 0%; 0%; 0%; 99.9%; 347°; 18°; 19°; 0%; 17°; 0°; 20°; 0°; 20°; 127°; 127°; 10461; 15878h; 21°; 75°
   *********************************************************************/
  std::string OutputHandler::formatValueDataForCSV(std::string time, std::list<VALUE_DATA> values)
  {
    std::ostringstream outStrStream;

    outStrStream << time << DELIMITER_FOR_CSV_FILE;

    for (auto iter = values.begin();
         iter != values.end();
         ++iter)
    {
      outStrStream << iter->value << DELIMITER_FOR_CSV_FILE;
    }
    outStrStream << std::endl;

    return outStrStream.str();
  }

  /*********************************************************************
   * @brief 	first step for creating a time series for the received radiator data
   *          ->  the radiator sends every second a "M2" command with the actual time
   *              followed by a "M1" command with the values at this time
   *          ->  in this function the "M2" is handled by storing the radiator time
   *              together with the microcontroller time and an empty values placeholder in the
   *              class member variable valuesAtTime
   *          ->  next function call must be the handling of the "M1" command -> see below
   * @param 	string with time received from radiator; e.g.    [TIME] Thu, 2022-09-08, 11:58:19
   * @return 	void
   *********************************************************************/
  void OutputHandler::handleValuesTimeSeries(std::string time)
  {
    LOG_info << millis() << " ms: OutputHandler::handleValuesTimeSeries -> TIME " << time << std::endl;

    valuesAtTime = std::make_tuple(millis(), time, emptyValuesPlaceholder);
  }

  /*********************************************************************
   * @brief 	second step for creating the values time series (more explanation see above)
   *          ->  adds values from "M1" command together with previous stored time data from "M2" command
   *              to class member deque valuesTimeSeries
   *          ->  expired items not needed anymore for later averaging etc.
   *              (older than fileOutputIntervallSec and MQTTOutputIntervallSec)
   *              are deleted (only one per function call)
   * @param 	list with values from radiator
   * @return 	void
   *********************************************************************/
  void OutputHandler::handleValuesTimeSeries(std::list<VALUE_DATA> values)
  {
    LOG_info << millis() << " ms: OutputHandler::handleValuesTimeSeries -> VALUES" << std::endl;

    // remove expired item (only one per function call -> should be enough)
    if (valuesTimeSeries.size() > 0)
    {
      ulong firstStoredTime = std::get<0>(valuesTimeSeries.front());
      auto deltaTime = millis() - firstStoredTime;
      if (deltaTime > (fileOutputIntervallSec * 1000) && deltaTime > (MQTTOutputIntervallSec * 1000))
      {
        valuesTimeSeries.pop_front();
      }
    }

    // store new values item
    std::get<2>(valuesAtTime) = values;
    valuesTimeSeries.push_back(valuesAtTime);

    // reset for later error recognition ...
    valuesAtTime = std::make_tuple(0, "0000-00-00, 00:00:00", emptyValuesPlaceholder);
  }

  /*********************************************************************
   * @brief 	get the last values item from the stored values time series
   * @param   Filter method to reduce data amount
   *          -> FilterMethod_t::DROP -> return newest values item from time series
   *          -> FilterMethod_t::AVERAGING -> not yet implemented
   *          -> FilterMethod_t::ONCHANGE -> not yet implemented
   * @param 	-> not used for filter method "DROP"
   *          -> only for "AVERAGING": time intervall for averaging of values
   *              -> if greater than stored item range -> all values are averaged
   * @return 	last values tuple
   *********************************************************************/
  OutputHandler::ValuesWithTime_t OutputHandler::getLastValuesAtTime(FilterMethod_t filterMethod, uint16_t intervallSec)
  {
    if (valuesTimeSeries.empty())
    {
      LOG_warn << "OutputHandler::getLastValuesAtTime: valuesTimeSeries is empty" << std::endl;
      return valuesAtTime; // only for error condition
    }

    switch (filterMethod)
    {
    case FilterMethod_t::ONCHANGE: // not yet implemented

      // TODO: checks values for change of defined amount

    case FilterMethod_t::AVERAGE:     // not yet implemented
      return valuesTimeSeries.back(); // like drop -> only as placeholder

      // TODO: averaging for all single values...
      //  auto storedTimeString = std::get<1>(valuesTimeSeries.back());
      //  for (auto rIter = valuesTimeSeries.rbegin(); // iterate backwards
      //       rIter != valuesTimeSeries.rend();
      //       ++rIter)
      //  {
      //  }

      break;
    case FilterMethod_t::DROP:
    default:
      return valuesTimeSeries.back();
      break;
    }
  }

  /*********************************************************************
   * @brief 	handle output of values to files
   *          -> inclusive check of fileOutputIntervallSec
   * @param 	used instance of Surveillance -> for access to parameter names
   * @return 	void
   *********************************************************************/
  void OutputHandler::handleValuesFileOutput(Surveillance &surveillance)
  {
    if (!toFile)
    {
      LOG_info << "Output to file is disabled" << std::endl;
      return;
    }

    static ulong nextFileOutputSec = 0;

    if (millis() / 1000 < nextFileOutputSec)
    {
      LOG_info << millis() << " ms: NO file output (next in "
               << (nextFileOutputSec - (millis() / 1000))
               << " seconds)" << std::endl;
      return;
    }

    nextFileOutputSec = millis() / 1000 + fileOutputIntervallSec;

    // static ulong lastFileOutput = 0;
    // if (millis() - lastFileOutput < fileOutputIntervallSec * 1000)
    // {
    //   LOG_info << millis() << " ms: NO file output (next in "
    //            << (fileOutputIntervallSec - (millis() - lastFileOutput) / 1000)
    //            << " seconds)" << std::endl;
    //   return;
    // }

    // lastFileOutput = millis();

    auto valuesAtTimeForFile = getLastValuesAtTime(FilterMethod_t::DROP);
    auto timeStringForValues = std::get<1>(valuesAtTimeForFile);
    auto valuesForFile = std::get<2>(valuesAtTimeForFile);
    auto outputForFile = formatValueDataForCSV(timeStringForValues, valuesForFile);

    auto filenameTillNow = outputFilename;
    if (handleFiles(deriveFilename(timeStringForValues)))
    {
      // check for new file to write the header only once
      if (filenameTillNow != outputFilename)
      {
        auto headerForNewFile = formatValueDataHeaderForCSV(surveillance);
        outputToFile(headerForNewFile + outputForFile);
      }
      else
        outputToFile(outputForFile); // without header
    }
    else
      LOG_error << "ERROR saving to File" << std::endl;
  }

  /*********************************************************************
   * @brief 	derive filename from input string
   * @param 	string of any length with included date in format yyyy-mm-dd (e.g. 2022-09-01)
   *          -> only the first contained date is used
   * @return 	filename derived from input string like 22-09-01.log
   *          -> or  00-LOST.log  if not date found in input string
   *********************************************************************/
  std::string OutputHandler::deriveFilename(std::string stringWithDate)
  {
    LOG_info << "OutputHandler::deriveFilename" << std::endl;

    // derive filename from string with integrated date in format   yyyy-mm-dd
    auto toFind = std::regex("(2[0-9]{3})-(0[1-9]|1[012])-([123]0|[012][1-9]|31)"); // finds date in string e.g.   2022-09-01
                                                                                    // https://www.softwaretestinghelp.com/regex-in-cpp/
                                                                                    // https://regexr.com

    std::string filename;
    std::smatch result;
    if (std::regex_search(stringWithDate, result, toFind))
    {
      // for (auto subres : result)
      //   LOG_info << "subres.str()" << subres.str() << std::endl;
      // filename = result[result.size() - 1].str() + ".log"; // use last match from stringWithDate

      filename = result.str() + ".log"; // use first match from stringWithDate
      LOG_info << "stringWithDate=" << stringWithDate
               << ", regex_search->result=" << result.str()
               << " ->filename=" << filename << std::endl;
    }
    else
    {
      filename = "00-LOST.log";
      LOG_warn << "No suitable date found in given stringWithDate for derivation of the filename ->  00-LOST.log   will be used" << std::endl;
    }

    return filename;
  }

  /*********************************************************************
   * @brief 	handle files for output stream
   * @param 	filename without complete path (e.g. 22-09-01.log)
   * @return 	true:  output file is opened and associated to class member outFileStream
   *          false: error opening file
   *********************************************************************/
  bool OutputHandler::handleFiles(std::string filename)
  {
    LOG_info << "OutputHandler::handleFiles" << std::endl;

    // same filename as before -> continue saving to it
    if (filename == outputFilename && outFileStream && outFileStream.good())
    {
      LOG_info << "The values will continue to be saved in file " << (outputPath + outputFilename) << std::endl;
      if (outFileStream.is_open())
        return true;
    }

    // if change of filename due to chage of date -> close the old file
    if (outFileStream.is_open())
    {
      outFileStream.close();
    }

    // new file or re-open previous file ...
    auto outputPathWithFilename = outputPath + filename;
    outFileStream.open(outputPathWithFilename, std::ofstream::out | std::ofstream::app);
    if (!outFileStream.is_open() || !outFileStream.good())
    {
      LOG_error << "ERROR opening file " << outputPathWithFilename << " !! NO VALUES ARE SAVED TO FILE !!" << std::endl;
      return false;
    }

    // new file
    if (outputFilename != filename)
    {
      // TODO: -> check for left space of filesystem and delete old files -> is now realized with independent parallel task
    }

    outputFilename = filename;

    LOG_info << "File " << outputPathWithFilename << " was opened" << std::endl;
    return true;
  }

  /*********************************************************************
   * @brief 	output string to a file which is already associated to outFileStream and opened
   * @param 	string with output data -> should be one line with eol (\n) termination
   * @param   interval (in seconds) to write from stream buffer to file (by closing the filestream)
   *          -> not every call of this function causes a file write operation,
   *             because the data is only held in the stream buffer!
   *          -> at failures (e.g. loss of energy) the data gets lost which is not written to file
   *          -> longer values (e.g. 15 min = 900sec) are reducing the load of the storage medium (espc. good for flash)
   * @return 	void
   *********************************************************************/
  void OutputHandler::outputToFile(std::string output, ulong writeToFileIntervalSec)
  {
    LOG_info << millis() << " ms: OutputHandler::outputToFile"
             << " -> ************* File output: *************\n"
             << output << std::endl;

    if (!outFileStream || !outFileStream.is_open() || !outFileStream.good())
    {
      LOG_error << millis() << " ms: Can NOT SAVE to file " << (outputPath + outputFilename)
                << "(outFileStream.is_open()=" << outFileStream.is_open()
                << ", outFileStream.good()=" << outFileStream.good() << ")"
                << std::endl;
      return;
    }

    outFileStream << output;

    // outFileStream.flush(); //with flush only -> nothing is written to file and data gets lost ...

    static ulong lastClose = 0;
    if (millis() - lastClose >= writeToFileIntervalSec * 1000)
    {
      outFileStream.close(); // BUG (with LittleFS only)? -> files are only written at .close() (.flush() is not working)
      lastClose = millis();
      LOG_info << lastClose << "ms: reached end of writeToFileIntervalSec=" << writeToFileIntervalSec << " -> Buffered file output data is written to file" << std::endl;
    }
    else
    {
      LOG_info << millis() << "ms: File output data buffered for next file write in" << ((millis() - lastClose) / 1000) << " seconds" << std::endl;
    }
  }

  void OutputHandler::outputErrorToBuzzer()
  {
    static TaskHandle_t Handle_xTaskBuzzer;
    static bool quitButtonWasPressed = false;

    if (!Handle_xTaskBuzzer)
    {
      auto xTaskBuzzer = [](void *parameter)
      {
        const auto buzzerPin = BUZZER_PIN;
        pinMode(buzzerPin, OUTPUT);
        auto beepIntervallMs = BEEP_INTERVALL_MS;
        bool OnOff = 1;

        const auto quitButtonPin = QUIT_BUZZER_BUTTON_PIN;
        pinMode(quitButtonPin, INPUT_PULLUP);
        quitButtonWasPressed = false;
        attachInterrupt(
            quitButtonPin,
            []() IRAM_ATTR
            {
              quitButtonWasPressed = true;
              detachInterrupt(quitButtonPin);
            },
            CHANGE);

        while (!quitButtonWasPressed)
        {
          digitalWrite(buzzerPin, OnOff);
          OnOff = !OnOff;
          vTaskDelay(pdMS_TO_TICKS(beepIntervallMs));
        }

        digitalWrite(buzzerPin, LOW);
        Handle_xTaskBuzzer = NULL;
        vTaskDelete(NULL);
      };

      // Create RTOS task
      BaseType_t _Result = xTaskCreatePinnedToCore(
          xTaskBuzzer,                      // Task function
          "xTask_Watchdog_and_Maintenance", // String with name of task
          3072,                             // Stack size in bytes
          NULL,                             // Parameter passed as input of the task
          uxTaskPriorityGet(NULL),          // Priority of the task: higher values -> higher priority
                                            // with uxTaskPriorityGet(NULL)-> same priority as current task
          &Handle_xTaskBuzzer,              // Task handle (Typ: TaskHandle_t)
          1);                               // Core 0 or 1 (Arduino code by default on Core 1)

      if (_Result != pdPASS)
        LOG_error << "outputErrorToBuzzer: Error creating xTask";
    }
  }

  /*********************************************************************
   * @brief 	prints the given outout to std::cout (usually Serial)
   * @param 	formatted string for output
   * @return 	void
   *********************************************************************/
  void OutputHandler::outputToConsole(std::string output)
  {
    std::cout << output;
  }

} // end namespace