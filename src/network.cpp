#include "network.h"
#include "debug.h"

#include <list>
#include <deque>
#include <regex>

#include <Preferences.h>

/*********************
 * STATIC DEFINITIONS
 *********************/
AsyncMqttClient radiator::NetworkHandler::mqttClient;
bool radiator::NetworkHandler::outputToMQTT = OUTPUT_TO_MQTT;
Ticker radiator::NetworkHandler::tickerReconnectMQTT;
Ticker radiator::NetworkHandler::tickerSendSysinfoToMQTT;
std::string radiator::NetworkHandler::mqttTopic = MQTT_TOPIC;
std::string radiator::NetworkHandler::mqttBroker = MQTT_BROKER;
std::string radiator::NetworkHandler::mqttUser = MQTT_USER;
std::string radiator::NetworkHandler::mqttPassword = MQTT_PASSWORD;
std::string radiator::NetworkHandler::bufStr;              // as class member to avoid heap fragmentation
std::ostringstream radiator::NetworkHandler::bufStrStream; // as class member to avoid heap fragmentation

/*********************************************************************
 * @brief 	Init NetworkHandler
 * @param 	true: enable output to MQTT
 *          false: disable output to MQTT
 * @param 	true: enable webserver
 *          false: disable webserver
 * @return 	void
 *********************************************************************/
bool radiator::NetworkHandler::init(const bool _outputToMQTT, const bool startWebServer)
{
  outputToMQTT = _outputToMQTT;
  bufStr.reserve(2000); // an attempt to prevent heap fragmentation

  installWiFiCallbacks();

  if (outputToMQTT)
    configureMQTT();

  if (!configureWiFiAndMQTT())
    return false;

#if START_WEBSERVER
  if (startWebServer)
    configureWebserver();
#endif

  return true;
}

/*********************************************************************
 * @brief 	Configure connection, disconnection, SSID and password settings for WiFi
 *          -> WiFi connection and reconnection attempts itself and made
 *             in a parallel background task from the WiFi class
 *          -> Change WiFi or MQTT settings from console/Serial by pressing the big yellow button
 *             (QUIT_BUZZER_BUTTON_PIN) at startup of the ESP32
 * @param 	void
 * @return 	true  -> settings are OK (->set persistent) and WiFi.begin is started in background
 *          false -> WiFi is NOT started,
 *                   because no persistent settings are available
 *                   OR WiFi is switched off as requested by user input
 *********************************************************************/
bool radiator::NetworkHandler::configureWiFiAndMQTT()
{
  // check if button on QUIT_BUZZER_BUTTON_PIN is pressed to start WiFiConfiguration
  pinMode(QUIT_BUZZER_BUTTON_PIN, INPUT_PULLUP);

  bool newWiFiConfig = false;

  if (!digitalRead(QUIT_BUZZER_BUTTON_PIN)) // LOW -> button is pressed for new configuration
  {
    std::cout << "\n\nWiFi: Button pressed at startup for WiFi and MQTT configuration:\n"
              << "\tNow scanning for WiFi networks ..."
              << std::endl;

    auto numberOfFoundNetworks = WiFi.scanNetworks(false); // scan in blocking mode

    std::cout << "Scan done -> found " << numberOfFoundNetworks << " WiFi networks:" << std::endl;

    for (int i = 0; i < numberOfFoundNetworks; i++)
    {
      std::cout << "[" << i + 1 << "] " << WiFi.SSID(i).c_str()
                << "\t RSSI=" << WiFi.RSSI(i)
                << std::endl;
    }

    auto oldTimeout = Serial.getTimeout();
    Serial.setTimeout(ULONG_MAX);

    int intInput;
    bool inputOk = false;

    do // ask for and verify input from user console
    {
      std::cout << "\t-> Select network by [number] \n\t-> or 0 to turn WiFi off \n"
                << "\t-> or -1 to skip and proceed with MQTT (actual WiFi settings will be used):    "
                << std::endl;
      intInput = Serial.parseInt();
      Serial.readStringUntil('\n').c_str(); // remove line end

      if (intInput < -1 || intInput > numberOfFoundNetworks)
        std::cout << "Input NOT VALID! Please do it again." << std::endl;
      else
        inputOk = true;

    } while (!inputOk);

    if (intInput == 0)
    {
      std::cout << "WiFi is turned OFF" << std::endl;
      WiFi.persistent(false);
      WiFi.disconnect(true, true);   // turn off WiFi, erase AP from persistent
      Serial.setTimeout(oldTimeout); // restore previous value
      if (Serial.available() > 0)    // clear buffer
        Serial.read();
      return false;
    }

    // std::string password;
    char password[64];

    if (intInput != -1)
    {
      std::cout << "Please input password for WLAN with SSID  " << WiFi.SSID(intInput - 1).c_str() << ":" << std::endl;
      // password = Serial.readStringUntil('\r').c_str();
      strncpy(password, Serial.readStringUntil('\r').c_str(), sizeof(password));
      Serial.readStringUntil('\n').c_str(); // remove line end

      std::cout << "SSID and password are stored persistent for automatic WiFi connection at startup" << std::endl;
      WiFi.persistent(true);
      newWiFiConfig = true;
    }

    outputToMQTT = inputMQTTconfig();

    if (newWiFiConfig)
      WiFi.begin(WiFi.SSID(intInput - 1).c_str(), password);
    // WiFi.begin(WiFi.SSID(intInput - 1).c_str(), password.c_str());

    Serial.setTimeout(oldTimeout); // restore previous value
    if (Serial.available() > 0)    // clear buffer
      Serial.read();
  }

  // button NOT pressed or only mqtt changes -> NO WiFi config change -> use persistent credentials
  if (!newWiFiConfig)
  {
    std::cout << "\n\nWiFi: Using persistent saved WiFi settings... \n"
              << "\t(Info: Change WiFi or MQTT settings by pressing the big yellow button at startup of the ESP32) \n"
              << std::endl;

    auto result = WiFi.begin();

    if (result == WL_CONNECT_FAILED)
    {
      std::cout << "WiFi connection failed due to missing persistent configuration" << std::endl;
      return false;
    }
  }

  WiFi.setHostname(WIFI_HOSTNAME);

  std::cout << millis() << " ms: WiFi: Try to connect now to WLAN in background task ..." << std::endl;

  return true;
}

/*********************************************************************
 * @brief 	ask user for new mqtt broker
 * @param 	void
 * @return 	true : use mqtt config
 *          false: mqtt should be turned off
 *********************************************************************/
bool radiator::NetworkHandler::inputMQTTconfig()
{
  loadMQTTConfigFromPreferences();
  std::cout << printMQTTConfig() << std::endl;

  std::cout << "Set MQTT broker (ENTER to use actual, 0 to delete and deactivate MQTT) HOSTNAME: " << std::endl;
  bufStr = Serial.readStringUntil('\r').c_str();
  RADIATOR_LOG_ERROR("strInput=" << bufStr << std::endl;)
  Serial.readStringUntil('\n').c_str(); // remove line end
  if (!bufStr.empty())
    if (bufStr == "0")
      mqttBroker = "";
    else
      mqttBroker = bufStr;

  std::cout << "Set MQTT user (ENTER to use actual, 0 to delete) USERNAME: " << std::endl;
  bufStr = Serial.readStringUntil('\r').c_str();
  RADIATOR_LOG_ERROR("strInput=" << bufStr << std::endl;)
  Serial.readStringUntil('\n').c_str(); // remove line end
  if (!bufStr.empty())
    if (bufStr == "0")
      mqttUser = "";
    else
      mqttUser = bufStr;

  std::cout << "Set MQTT password (ENTER to use actual, 0 to delete) PASSWORD: " << std::endl;
  bufStr = Serial.readStringUntil('\r').c_str();
  RADIATOR_LOG_ERROR("strInput=" << bufStr << std::endl;)
  Serial.readStringUntil('\n').c_str(); // remove line end
  if (!bufStr.empty())
    if (bufStr == "0")
      mqttPassword = "";
    else
      mqttPassword = bufStr;

  Preferences mqttPrefs;
  mqttPrefs.begin(PREFERENCES_NAMESPACE);
  mqttPrefs.putString("mqttBroker", mqttBroker.c_str());
  mqttPrefs.putString("mqttUser", mqttUser.c_str());
  mqttPrefs.putString("mqttPassword", mqttPassword.c_str());
  mqttPrefs.end();

  RADIATOR_LOG_INFO("Saved mqttBroker hostname to preferences in NVS" << std::endl;)

  if (mqttBroker.empty())
    return false;

  configureMQTT();

  return true;

  // with regex check of hostname:
  //  auto toFind = std::regex("([a-zA-Z0-9-.]{2,256}\\.[a-z]{2,5})"); // finds url in string e.g.   broker.hivemq.com   or homeassistant.local
  //                                                                   // https://regexr.com
  //  std::smatch result;

  // std::cout << printMQTTConfig() << std::endl;
  // do
  // {
  //   std::cout << "Set MQTT broker (ENTER to use actual) hostname: " << std::endl;

  //   bufStr = Serial.readStringUntil('\r').c_str();
  //   auto res = std::regex_search(bufStr, result, toFind);
  //   RADIATOR_LOG_ERROR("strInput=" << bufStr << ", regex_search->result=" << result.str() << std::endl;)

  // } while (result.empty() && bufStr.size() > 1);

  // if (!result.empty())
  // {
  //   mqttBroker = result.str();

  //   Preferences mqttPrefs;
  //   mqttPrefs.begin(PREFERENCES_NAMESPACE);
  //   mqttPrefs.putString("mqttBroker", mqttBroker.c_str());
  //   mqttPrefs.end();

  //   RADIATOR_LOG_INFO("Saved mqttBroker hostname to preferences in NVS" << std::endl;)

  //   configureMQTT();
  //}
}

/*********************************************************************
 * @brief 	install needed WiFi callbacks as Lambda's
 * @param 	void
 * @return 	void
 *********************************************************************/
void radiator::NetworkHandler::installWiFiCallbacks()
{
  // on connection
  WiFi.onEvent(
      [](WiFiEvent_t _WiFi_Event, WiFiEventInfo_t _WiFi_Event_Info)
      {
        bufStr = "\nWiFi connected to... \n" +
                 (std::string) "\t WLAN / SSID: \t" + WiFi.SSID().c_str() + "\n" +
                 "\t Signal: \tRSSI " + std::to_string(WiFi.RSSI()) + "\n" +
                 "\t Hostname: \t" + WiFi.getHostname() + "\n" +
                 "\t IP address: \t" + WiFi.localIP().toString().c_str() + "\n" +
                 "\t Gateway IP: \t" + WiFi.gatewayIP().toString().c_str() + "\n" +
                 "\t Network IP: \t" + WiFi.networkID().toString().c_str() + "\n";
        RADIATOR_LOG_WARN(bufStr << std::endl;)

        if (REDIRECT_STD_ERR_TO_SYSLOG_FILE)
          std::cout << bufStr << std::endl; // for better user info also to console

        // make some noise to signal connection ;-) ...
        pinMode(BUZZER_PIN, OUTPUT);
        bool OnOff = 1;
        for (int i = 0; i < 5; i++)
        {
          digitalWrite(BUZZER_PIN, OnOff);
          OnOff = !OnOff;
          vTaskDelay(pdMS_TO_TICKS(100));
        }
        digitalWrite(BUZZER_PIN, LOW);
      },
      WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);

  // on disconnection
  WiFi.onEvent(
      [](WiFiEvent_t _WiFi_Event, WiFiEventInfo_t _WiFi_Event_Info)
      {
        static ulong timeConnectionLostMs = 0;

        if (WiFi.status() == WL_CONNECTION_LOST)
        {
          timeConnectionLostMs = millis();
          bufStr = "\nWiFi was DISCONNECTED from WLAN. System is trying to reconnect in a background task ... \n"
                   "(Info: Change WiFi or MQTT settings by pressing the big yellow button at startup of the ESP32)\n";
          RADIATOR_LOG_WARN(bufStr << std::endl;)

          if (REDIRECT_STD_ERR_TO_SYSLOG_FILE)
            std::cout << bufStr << std::endl; // for better user info also to console
        }

        static ulong nextInfoOutputMs = 0;
        static const int infoOutputIntervallSec = 15 * 60; // info output all 15 minutes -> consider needed space when output is redirected to syslog file

        if (WiFi.status() == WL_NO_SSID_AVAIL && millis() >= nextInfoOutputMs)
        {
          bufStr = "\nWiFi is NOT CONNECTED to WLAN -> SSID not available. The system keeps trying to reconnect in a background task ... \n"
                   "\t(Connection was lost " +
                   std::to_string((millis() - timeConnectionLostMs) / (1000 * 60)) +
                   " minutes ago.)\n"
                   "\t(Info: Change WiFi or MQTT settings by pressing the big yellow button at startup of the ESP32)\n";
          RADIATOR_LOG_INFO(bufStr << std::endl;)

          if (REDIRECT_STD_ERR_TO_SYSLOG_FILE)
            std::cout << bufStr << std::endl; // for better user info also to console

          nextInfoOutputMs = millis() + infoOutputIntervallSec * 1000;

          // make some noise
          pinMode(BUZZER_PIN, OUTPUT);
          bool OnOff = 1;
          for (int i = 0; i < 3; i++)
          {
            digitalWrite(BUZZER_PIN, OnOff);
            OnOff = !OnOff;
            vTaskDelay(pdMS_TO_TICKS(750));
          }
          digitalWrite(BUZZER_PIN, LOW);
        }
      },
      WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  // on got IP
  WiFi.onEvent(
      [](WiFiEvent_t _WiFi_Event, WiFiEventInfo_t _WiFi_Event_Info)
      {
        bufStr = "\nWiFi:\tGot IP address:\t" + (std::string)WiFi.localIP().toString().c_str() + "\n" +
                 "\tGateway IP: \t" + WiFi.gatewayIP().toString().c_str() + "\n" +
                 "\tNetwork IP: \t" + WiFi.networkID().toString().c_str() + "\n";

        RADIATOR_LOG_WARN(bufStr << std::endl;)

        if (REDIRECT_STD_ERR_TO_SYSLOG_FILE)
          std::cout << bufStr << std::endl; // for better user info also to console

        if (outputToMQTT)
          connectToMqtt();
      },
      WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
}

/*********************************************************************
 * @brief 	load MQTT config from preferences
 * @param 	void
 * @return 	void
 *********************************************************************/
void radiator::NetworkHandler::loadMQTTConfigFromPreferences()
{
  // load from preferences-flash
  Preferences mqttPrefs;
  mqttPrefs.begin(PREFERENCES_NAMESPACE);
  mqttBroker = mqttPrefs.getString("mqttBroker", MQTT_BROKER).c_str();
  mqttUser = mqttPrefs.getString("mqttUser", MQTT_USER).c_str();
  mqttPassword = mqttPrefs.getString("mqttPassword", MQTT_PASSWORD).c_str();
  mqttPrefs.end();
}

/*********************************************************************
 * @brief 	configure MQTT AsyncMqttClient
 * @param 	void
 * @return 	void
 *********************************************************************/
void radiator::NetworkHandler::configureMQTT()
{
  loadMQTTConfigFromPreferences();

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);

  mqttClient.setServer(mqttBroker.c_str(), MQTT_PORT);

  // if (!mqttUser.empty() && !mqttPassword.empty())
  mqttClient.setCredentials(mqttUser.c_str(), mqttPassword.c_str());

  mqttClient.setKeepAlive(MQTT_KEEP_ALIVE);
  mqttClient.setCleanSession(false); // hold queued messages after disconnect -> consider needed ram at longer disconnection intervals

  static std::string lastWillTopic = mqttTopic + MQTT_SUBTOPIC_ONLINESTATUS;
  static std::string lastWill = (std::string)mqttClient.getClientId() + ": offline";
  mqttClient.setWill(lastWillTopic.c_str(), 1, true, lastWill.c_str()); // topic and payload for lastWill must be defined static!!

  std::cout << printMQTTConfig() << std::endl;

  if (mqttBroker.empty())
  {
    RADIATOR_LOG_ERROR(millis() << " ms: NO MQTT broker address -> MQTT is deactivated! To activate push the big yellow button at startup of ESP" << std::endl;)
    outputToMQTT = false;
    // return;
  }
}

/*********************************************************************
 * @brief 	print config info about mqtt client
 * @param 	void
 * @return 	formatted infos as string
 *********************************************************************/
std::string radiator::NetworkHandler::printMQTTConfig()
{
  return "MQTT config: \t Broker= " + mqttBroker +
         ", user= " + mqttUser +
         ", password= " + mqttPassword +
         ", Port= " + std::to_string(MQTT_PORT) +
         ", KeepAlive= " + std::to_string((int)MQTT_KEEP_ALIVE) + " sec \n" +
         "\t\t ClientID= " + mqttClient.getClientId() +
         ", Topic= " + mqttTopic;
}

/*********************************************************************
 * @brief 	callback for MQTT connection
 * @param 	session present
 * @return 	void
 *********************************************************************/
void radiator::NetworkHandler::onMqttConnect(bool sessionPresent)
{
  bufStr = std::to_string(millis()) + " ms: Connected to MQTT: Broker= " + mqttBroker +
           ", Session present= " + std::to_string(sessionPresent) +
           ", mqttClient.getClientId()=" + mqttClient.getClientId();
  RADIATOR_LOG_WARN(bufStr << std::endl;)

  if (REDIRECT_STD_ERR_TO_SYSLOG_FILE)
    std::cout << bufStr << std::endl; // for better user info also to console

  bufStr = std::to_string(millis()) + " ms: " + (std::string)mqttClient.getClientId() + ": online";
  mqttClient.publish((mqttTopic + MQTT_SUBTOPIC_ONLINESTATUS).c_str(), 1, true, bufStr.c_str());
  mqttClient.publish((mqttTopic + MQTT_SUBTOPIC_SYSINFO).c_str(), 1, true, get_System_Info().c_str());

  if (!tickerSendSysinfoToMQTT.active())
  {
    tickerSendSysinfoToMQTT.attach_ms( // send sysinfo periodic to mqtt for system health analysis
        MQTT_INTERVALL_FOR_SYSINFO_SEC * 1000,
        []()
        {
          mqttClient.publish((mqttTopic + MQTT_SUBTOPIC_SYSINFO).c_str(), 1, true, get_System_Info().c_str());
          // RADIATOR_LOG_WARN( get_System_Info();
          RADIATOR_LOG_WARN(millis() << " ms: handleValuesTimeSeries: Uptime: " << (millis() / (1000 * 60)) << " min; MQTT status: " << (mqttClient.connected() ? "connected" : "NOT connected") << "; Heap : " << ESP.getFreeHeap() << " / " << ESP.getMinFreeHeap() << " / " << ESP.getMaxAllocHeap() << std::endl;)
        });
  }

  // mqttClient.subscribe("/#", 1);
}

/*********************************************************************
 * @brief 	callback for MQTT disconnection
 * @param 	reason for disconnection
 * @return 	void
 *********************************************************************/
void radiator::NetworkHandler::onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  bufStr = std::to_string(millis()) + " ms: Disconnected from MQTT: reason = ";

  switch (reason)
  {
  case AsyncMqttClientDisconnectReason::TCP_DISCONNECTED:
    bufStr += "TCP_DISCONNECTED";
    break;
  case AsyncMqttClientDisconnectReason::MQTT_UNACCEPTABLE_PROTOCOL_VERSION:
    bufStr += "MQTT_UNACCEPTABLE_PROTOCOL_VERSION";
    break;
  case AsyncMqttClientDisconnectReason::MQTT_IDENTIFIER_REJECTED:
    bufStr += "MQTT_IDENTIFIER_REJECTED";
    break;
  case AsyncMqttClientDisconnectReason::MQTT_SERVER_UNAVAILABLE:
    bufStr += "MQTT_SERVER_UNAVAILABLE";
    break;
  case AsyncMqttClientDisconnectReason::MQTT_MALFORMED_CREDENTIALS:
    bufStr += "MQTT_MALFORMED_CREDENTIALS";
    break;
  case AsyncMqttClientDisconnectReason::MQTT_NOT_AUTHORIZED:
    bufStr += "MQTT_NOT_AUTHORIZED";
    break;
  case AsyncMqttClientDisconnectReason::ESP8266_NOT_ENOUGH_SPACE:
    bufStr += "ESP8266_NOT_ENOUGH_SPACE";
    break;
  case AsyncMqttClientDisconnectReason::TLS_BAD_FINGERPRINT:
    bufStr += "TLS_BAD_FINGERPRINT";
    break;
  default:
    bufStr += "UNKNOWN";
    break;
  }

  RADIATOR_LOG_WARN(bufStr << std::endl;)

  if (REDIRECT_STD_ERR_TO_SYSLOG_FILE)
    std::cout << bufStr << std::endl; // for better user info also to console

  if (WiFi.isConnected() && outputToMQTT)
  {
    bufStr = std::to_string(millis()) + " ms: Start ticker for reconnect to MQTT broker in " + std::to_string(MQTT_RECONNECTION_TIMEOUT_SEC) + " sec.";
    RADIATOR_LOG_INFO(bufStr << std::endl;)

    if (REDIRECT_STD_ERR_TO_SYSLOG_FILE)
      std::cout << bufStr << std::endl; // for better user info also to console

    tickerReconnectMQTT.once(MQTT_RECONNECTION_TIMEOUT_SEC, connectToMqtt);
  }
}

/*********************************************************************
 * @brief 	callback for MQTT topic subscription
 * @param 	packet ID
 * @param   Quality of Service (0, 1 or 2)
 * @return 	void
 *********************************************************************/
void radiator::NetworkHandler::onMqttSubscribe(uint16_t packetId, uint8_t qos)
{
  RADIATOR_LOG_INFO(millis() << " ms: Subscribe acknowledged: packetId= " << packetId << ", qos= " << (int)qos << std::endl;)
}

/*********************************************************************
 * @brief 	callback for MQTT topic UNsubscription
 * @param 	packet ID
 * @return 	void
 *********************************************************************/
void radiator::NetworkHandler::onMqttUnsubscribe(uint16_t packetId)
{
  RADIATOR_LOG_INFO(millis() << " ms: Unsubscribe acknowledged: packetId= " << packetId << std::endl;)
}

/*********************************************************************
 * @brief 	callback for received MQTT message
 *          (-> not used in this application ...)
 * @param 	topic e.g. "Heizung_Froeling_P2/status"
 * @param 	received payload
 * @param 	message properties
 * @param 	payload lenght
 * @param 	index
 * @param 	total message size
 * @return  void
 *********************************************************************/
void radiator::NetworkHandler::onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
  RADIATOR_LOG_INFO(millis()
                        << " ms: Published message received: \n"
                        << "\t topic= " << topic << "\n"
                        << "\t payload= " << payload << "\n"
                        << "\t qos= " << (int)properties.qos << "\n"
                        << "\t dup= " << properties.dup << "\n"
                        << "\t retain= " << properties.retain << "\n"
                        << "\t len= " << len << "\n"
                        << "\t index= " << index << "\n"
                        << "\t total= " << total
                        << std::endl;)
}

/*********************************************************************
 * @brief 	callback for MQTT publish from client
 * @param 	packet ID
 * @return 	void
 *********************************************************************/
void radiator::NetworkHandler::onMqttPublish(uint16_t packetId)
{
  RADIATOR_LOG_INFO(millis() << " ms: MQTT Publish acknowledged for packetId= " << packetId << std::endl;)
}

/*********************************************************************
 * @brief 	establish connection to previously configured MQTT broker
 * @param 	void
 * @return 	void
 *********************************************************************/
void radiator::NetworkHandler::connectToMqtt()
{
  if (!outputToMQTT)
  {
    RADIATOR_LOG_WARN(millis() << " ms: Output to MQTT is disabled by config (outputToMQTT=false)" << std::endl;)
    return;
  }

  if (mqttClient.connected())
  {
    RADIATOR_LOG_INFO(millis() << " ms: MQTT client is already connected" << std::endl;)
    return;
  }

  // mqttClient.clearQueue();

  RADIATOR_LOG_INFO(millis() << " ms: Connecting to MQTT broker    " << mqttBroker << std::endl;)
  mqttClient.connect();
}

/*********************************************************************
 * @brief 	disconnects MQTT from broker
 * @param 	void
 * @return 	void
 *********************************************************************/
void radiator::NetworkHandler::disconnectMqtt()
{
  RADIATOR_LOG_INFO(millis() << " ms: DISconnecting MQTT: client ID=" << mqttClient.getClientId() << std::endl;)

  if (!mqttClient.connected())
  {
    RADIATOR_LOG_INFO(millis() << " ms: ... MQTT client is NOT connected" << std::endl;)
    return;
  }

  mqttClient.disconnect();
}

/*********************************************************************
 * @brief 	publish message at predefined topic to previous configured MQTT broker
 * @param 	message (=payload)
 * @param 	optional subtopic e.g. "/status" (default "")
 * @param 	optional quality of service (0, 1 or 2) (default 1)
 * @param 	optional retain true/false (default true)
 * @return 	0   : failure at publishing
 *          >=1 : published packetID
 *********************************************************************/
bool radiator::NetworkHandler::publishToMQTT(const std::string &payload, const std::string &subtopic, const uint8_t qos, const bool retain)
{
  if (!outputToMQTT)
  {
    RADIATOR_LOG_WARN(millis() << " ms: Output to MQTT is disabled by config" << std::endl;)
    return false;
  }

  // mqttClient itself buffers only until "official" disconnection message from MQTT broker
  // - so we have to buffer and resend the data on our own
  struct BuffStruct
  {
    std::string payload;
    std::string subtopic;
    uint8_t qos;
    bool retain;
  };
  static std::deque<BuffStruct> bufferQueue;

  // we need a mutex semaphore to handle concurrent access from different tasks - otherwise "funny" crashes from bufferQueue-handling
  static SemaphoreHandle_t semaphoreMqttPublish = xSemaphoreCreateMutex();
  xSemaphoreTake(semaphoreMqttPublish, portMAX_DELAY);

  // ensure a size limit for buffered data by conditional deleting of buffer items
  while (ESP.getMaxAllocHeap() < MIN_FREE_HEAPSIZE_FOR_MQTT_BUFFERQUEUE_BYTES) // ESP.getFreeHeap() or ESP.getMinFreeHeap() or ESP.getMaxAllocHeap()
  {
    RADIATOR_LOG_WARN("Old MQTT messages cannot be buffered any more due to low free heap size: ESP.getFreeHeap()="
                          << ESP.getFreeHeap() << " bytes, ESP.getMinFreeHeap()=" << ESP.getMinFreeHeap() << " bytes, ***ESP.getMaxAllocHeap()=" << ESP.getMaxAllocHeap() << " bytes***, "
                          << " -> oldest buffered message will be deleted (bufferQueue.size=" << bufferQueue.size() << " elements)" << std::endl;)

    if (bufferQueue.empty()) // avoid endless loop when deleting buffer elements here did not release enough heap space ...
      break;

    bufferQueue.pop_front();
  }

  // ALL messages are pushed to the buffer queue
  RADIATOR_LOG_DEBUG(millis() << " ms: publishToMQTT: 1 before push_back-> bufferQueue.size=" << bufferQueue.size() << " ESP.getFreeHeap()=" << ESP.getFreeHeap() << " / " << ESP.getMinFreeHeap() << " / " << ESP.getMaxAllocHeap() << " bytes" << std::endl;)
  bufferQueue.push_back({payload, subtopic, qos, retain});
  RADIATOR_LOG_DEBUG(millis() << " ms: publishToMQTT: 2 after push_back-> bufferQueue.size=" << bufferQueue.size() << " ESP.getFreeHeap()=" << (ESP.getFreeHeap()) << " / " << ESP.getMinFreeHeap() << " / " << ESP.getMaxAllocHeap() << " bytes" << std::endl;)

  // DEBUG_STACK_HIGH_WATERMARK

  if (!mqttClient.connected())
  {
    RADIATOR_LOG_INFO(millis() << " ms: Cannot publish to MQTT broker (" << mqttBroker << ") -> mqttClient is not connected (WiFi.isConnected()="
                               << WiFi.isConnected() << ")" << std::endl;)
    RADIATOR_LOG_DEBUG("\t -> message is buffered (queue size=" << bufferQueue.size()
                                                                << ", ESP.getFreeHeap()=" << (ESP.getFreeHeap()) << " / " << ESP.getMinFreeHeap() << " / " << ESP.getMaxAllocHeap()
                                                                << ") and send at reconnection \n"
                                                                << "\t topic = " << (mqttTopic + bufferQueue.back().subtopic) << "\n"
                                                                << "\t payload = " << bufferQueue.back().payload << std::endl;)

    xSemaphoreGive(semaphoreMqttPublish);

    return false;
  }

  uint16_t packetId;

  // send ALL buffered messages
  while (!bufferQueue.empty())
  {
    packetId = mqttClient.publish((mqttTopic + bufferQueue.front().subtopic).c_str(),
                                  bufferQueue.front().qos,
                                  bufferQueue.front().retain,
                                  bufferQueue.front().payload.c_str());

    RADIATOR_LOG_INFO(millis() << " ms: publishToMQTT: broker = " << mqttBroker
                               << ", topic = " << (mqttTopic + bufferQueue.front().subtopic)
                               << ", Quality of service = " << (int)bufferQueue.front().qos
                               << ", retain = " << bufferQueue.front().retain << ", packetId = " << packetId
                               << ", \n\tpayload=\n\t" << bufferQueue.front().payload << std::endl;)

    bufferQueue.pop_front(); // remove sent message from buffer queue
  }

  xSemaphoreGive(semaphoreMqttPublish);

  return packetId; // 0 on failure
}

/*********************************************************************
 * @brief 	assemble and format system info to a std::string
 * @param   void
 * @return 	systeminfo
 *********************************************************************/
std::string radiator::NetworkHandler::get_System_Info()
{
  RADIATOR_LOG_INFO("NetworkHandler::get_System_Info()" << std::endl;)

  // Version 1 with local std::stringstream object -> watch for heap fragmentation!
  // std::stringstream systemInfo;
  bufStrStream.str(""); // empty streambuffer

  // systemInfo
  bufStrStream
      << millis() << " ms: Microcontroller ESP32: \n"
      << "ESP32 chip model   : " << ESP.getChipModel() << "; \n"
      << "Chip Revision      : " << ((int)ESP.getChipRevision()) << "; \n"
      << "CPU Frequency      : " << ESP.getCpuFreqMHz() << " MHz; \n"
      << "Chip cores         : " << ((int)ESP.getChipCores()) << "; \n"
      << "Flash chip size    : " << (ESP.getFlashChipSize() / 1024) << " kBytes; \n"
      << "Flash chip speed   : " << (ESP.getFlashChipSpeed() / 1000000) << " MHz; \n"
      << "Sketch size        : " << (ESP.getSketchSize() / 1024) << " kBytes; \n"
      << "Free sketch space  : " << (ESP.getFreeSketchSpace() / 1024) << " kBytes; \n"
      << "SDK version        : " << ESP.getSdkVersion() << "; \n"

      << "\n"

      << "Uptime             : " << (millis() / (1000 * 60)) << " min; \n"
      << "Heapsize           : " << (ESP.getHeapSize() / 1024) << " kBytes; \n"
      << "Free heap          : " << (ESP.getFreeHeap() / 1024) << " kBytes; \n"
      << "Min. free heap     : " << (ESP.getMinFreeHeap() / 1024) << " kBytes; \n"
      << "Max. alloc heap    : " << (ESP.getMaxAllocHeap() / 1024) << " kBytes; \n"
      << "Free PSRAM         : " << ESP.getFreePsram() << " Bytes; \n"

      << "\n"

      << "WiFi IP            : " << WiFi.localIP().toString().c_str() << "; \n"
      << "WiFi hostname      : " << WiFi.getHostname() << "; \n"
      << "WiFi SSID          : " << WiFi.SSID().c_str() << "; \n"
      << "WiFi status        : " << WiFi.status() << "; \n"
      << "WiFi strength RSSI : " << ((int)WiFi.RSSI()) << " dB; \n"
      << "WiFi MAC           : " << WiFi.macAddress().c_str() << "; \n"
      << "WiFi subnet        : " << WiFi.subnetMask().toString().c_str() << "; \n"
      << "WiFi gateway       : " << WiFi.gatewayIP().toString().c_str() << "; \n"
      << "WiFi DNS 1         : " << WiFi.dnsIP(0).toString().c_str() << "; \n"
      << "WiFi DNS 2         : " << WiFi.dnsIP(1).toString().c_str() << "; \n"
      << "Webserver          : " << (START_WEBSERVER ? "Started" : "NOT started") << "; \n"

      << "\n"

      << "Filesystem               : " << XSTRINGIFY(FILESYSTEM_TO_USE) << "; \n"
      << "Basepath(Mountpoint)     : " << FILESYSTEM_BASE_PATH << "; \n"
      << "Filesystem total         : " << (FILESYSTEM_TO_USE.totalBytes() / 1024) << " kBytes; \n"
      << "Used Filesystem          : " << (FILESYSTEM_TO_USE.usedBytes() / 1024) << " kBytes; \n"
      << "Free Filesystem          : " << ((FILESYSTEM_TO_USE.totalBytes() - FILESYSTEM_TO_USE.usedBytes()) / 1024) << " kBytes; \n"

      << "\n"

      << "Data directory           : " << DATA_DIRECTORY << "; \n"
      << "File output interval     : " << FILE_OUTPUT_INTERVALL_SEC << " sec; \n"
      << "Write to file interval   : " << WRITE_TO_FILE_INTERVAL_SEC << " sec; \n"
      << "Filesystem check interval: " << INTERVALL_FOR_FILESYSTEM_CHECK_SEC << " sec; \n"

      << "\n"

      << "Redirect to syslog file  : " << (REDIRECT_STD_ERR_TO_SYSLOG_FILE ? "On" : "Off") << "; \n"
      << "Syslog filename          : " << SYSLOG_PATHNAME << " \n"
      << "Write to syslog interval : " << WRITE_TO_SYSLOGFILE_INTERVAL_SEC << " sec; \n"
      << "Syslog max. filesize     : " << MAX_SYSLOG_FILE_SIZE_BYTES << " bytes; \n"
      << "Syslog max. files        : " << MAX_OLD_SYSLOG_FILES << "; \n"

      << "\n"

      << "MQTT output              : " << (outputToMQTT ? "activated" : "NOT activated") << "; \n"
      << "MQTT broker              : " << mqttBroker << "; \n"
      << "MQTT port                : " << MQTT_PORT << "; \n"
      << "MQTT status              : " << (mqttClient.connected() ? "connected" : "NOT connected") << "; \n"
      << "MQTT client ID           : " << mqttClient.getClientId() << "; \n"
      << "MQTT topic               : " << mqttTopic << "; \n"
      << "MQTT syslog subtopic     : " << MQTT_SUBTOPIC_SYSLOG << "; \n"
      << "MQTT output interval     : " << MQTT_OUTPUTINTERVALL_SEC << " sec; \n"
      << "MQTT keep alive          : " << MQTT_KEEP_ALIVE << " sec; \n"
      << "MQTT reconnection timeout: " << MQTT_RECONNECTION_TIMEOUT_SEC << " sec; \n"

      << std::endl;

  // return systemInfo.str();
  return bufStrStream.str();

  // // Version 2 with std::string: -> sometimes memory problems in ticker
  // // return std::to_string(millis()) +
  // bufStr.reserve(2000);
  // bufStr = std::to_string(millis()) +
  //          " ms: Microcontroller ESP32: \n" +
  //          "ESP32 chip model   : " + ESP.getChipModel() + "; \n" +
  //          "Chip Revision      : " + std::to_string(ESP.getChipRevision()) + "; \n" +
  //          "CPU Frequency      : " + std::to_string(ESP.getCpuFreqMHz()) + " MHz; \n" +
  //          "Chip cores         : " + std::to_string(ESP.getChipCores()) + "; \n" +
  //          "Flash chip size    : " + std::to_string(ESP.getFlashChipSize() / 1024) + " kBytes; \n" +
  //          "Flash chip speed   : " + std::to_string(ESP.getFlashChipSpeed() / 1000000) + " MHz; \n" +
  //          "Sketch size        : " + std::to_string(ESP.getSketchSize() / 1024) + " kBytes; \n" +
  //          "Free sketch space  : " + std::to_string(ESP.getFreeSketchSpace() / 1024) + " kBytes; \n" +
  //          "SDK version        : " + ESP.getSdkVersion() + "; \n" +

  //          "\n" +

  //          "Uptime             : " + std::to_string(millis() / (1000 * 60)) + " min; \n" +
  //          "Heapsize           : " + std::to_string(ESP.getHeapSize() / 1024) + " kBytes; \n" +
  //          "Free heap          : " + std::to_string(ESP.getFreeHeap() / 1024) + " kBytes; \n" +
  //          "Min. free heap     : " + std::to_string(ESP.getMinFreeHeap() / 1024) + " kBytes; \n" +
  //          "Max. alloc heap    : " + std::to_string(ESP.getMaxAllocHeap() / 1024) + " kBytes; \n" +
  //          "Free PSRAM         : " + std::to_string(ESP.getFreePsram()) + " Bytes; \n"

  //          + "\n" +

  //          +"WiFi IP            : " + WiFi.localIP().toString().c_str() + "; \n" +
  //          "WiFi hostname      : " + WiFi.getHostname() + "; \n" +
  //          "WiFi SSID          : " + WiFi.SSID().c_str() + "; \n" +
  //          "WiFi status        : " + std::to_string(WiFi.status()) + "; \n" +
  //          "WiFi strength RSSI : " + std::to_string(WiFi.RSSI()) + " dB; \n" +
  //          "WiFi MAC           : " + WiFi.macAddress().c_str() + "; \n" +
  //          "WiFi subnet        : " + WiFi.subnetMask().toString().c_str() + "; \n" +
  //          "WiFi gateway       : " + WiFi.gatewayIP().toString().c_str() + "; \n" +
  //          "WiFi DNS 1         : " + WiFi.dnsIP(0).toString().c_str() + "; \n" +
  //          "WiFi DNS 2         : " + WiFi.dnsIP(1).toString().c_str() + "; \n" +
  //          "Webserver          : " + (START_WEBSERVER ? "Started" : "NOT started") + "; \n"

  //          + "\n" +

  //          "Filesystem               : " + (std::string)(XSTRINGIFY(FILESYSTEM_TO_USE)) + "; \n" +
  //          "Basepath(Mountpoint)     : " + FILESYSTEM_BASE_PATH + "; \n" +
  //          "Filesystem total         : " + std::to_string(FILESYSTEM_TO_USE.totalBytes() / 1024) + " kBytes; \n" +
  //          "Used Filesystem          : " + std::to_string(FILESYSTEM_TO_USE.usedBytes() / 1024) + " kBytes; \n" +
  //          "Free Filesystem          : " + std::to_string((FILESYSTEM_TO_USE.totalBytes() - FILESYSTEM_TO_USE.usedBytes()) / 1024) + " kBytes; \n"

  //          + "\n" +

  //          "Data directory           : " + DATA_DIRECTORY + "; \n" +
  //          "File output interval     : " + std::to_string(FILE_OUTPUT_INTERVALL_SEC) + " sec; \n" +
  //          "Write to file interval   : " + std::to_string(WRITE_TO_FILE_INTERVAL_SEC) + " sec; \n" +
  //          "Filesystem check interval: " + std::to_string(INTERVALL_FOR_FILESYSTEM_CHECK_SEC) + " sec; \n"

  //          + "\n" +

  //          "Redirect to syslog file  : " + (REDIRECT_STD_ERR_TO_SYSLOG_FILE ? "On" : "Off") + "; \n" +
  //          "Syslog filename          : " + SYSLOG_PATHNAME + " \n" +
  //          "Write to syslog interval : " + std::to_string(WRITE_TO_SYSLOGFILE_INTERVAL_SEC) + " sec; \n" +
  //          "Syslog max. filesize     : " + std::to_string(MAX_SYSLOG_FILE_SIZE_BYTES) + " bytes; \n" +
  //          "Syslog max. files        : " + std::to_string(MAX_OLD_SYSLOG_FILES) + "; \n"

  //          + "\n" +

  //          "MQTT output              : " + (outputToMQTT ? "activated" : "NOT activated") + "; \n" +
  //          "MQTT broker              : " + mqttBroker + "; \n" +
  //          "MQTT port                : " + std::to_string(MQTT_PORT) + "; \n" +
  //          "MQTT status              : " + (mqttClient.connected() ? "connected" : "NOT connected") + "; \n" +
  //          "MQTT client ID           : " + mqttClient.getClientId() + "; \n" +
  //          "MQTT topic               : " + mqttTopic + "; \n" +
  //          "MQTT syslog subtopic     : " + MQTT_SUBTOPIC_SYSLOG + "; \n" +
  //          "MQTT output interval     : " + std::to_string(MQTT_OUTPUTINTERVALL_SEC) + " sec; \n" +
  //          "MQTT keep alive          : " + std::to_string(MQTT_KEEP_ALIVE) + " sec; \n" +
  //          "MQTT reconnection timeout: " + std::to_string(MQTT_RECONNECTION_TIMEOUT_SEC) + " sec; \n";

  // return bufStr;
}

#if START_WEBSERVER
/*********************************************************************
 * @brief 	configure Webserver
 * @param 	void
 * @return 	void
 *********************************************************************/
void radiator::NetworkHandler::configureWebserver()
{
  static AsyncWebServer server(80); // Create AsyncWebServer object on port 80

  // <link rel = "icon" href = "data:,">

  // create very simple html page
  static const char index_html[] PROGMEM =
      R"rawliteral(
        <!DOCTYPE HTML><html>
        <head>
          <title>Pelletsheizung Froeling P2/S3100</title>
          <meta charset="UTF-8">
          <meta name="viewport" content="width=device-width, initial-scale=1.0">
          <link rel="icon" href="/favicon.ico" type="image/x-icon">
          <style>
          </style>
        </head>
        <body>
          <h2>ESP Web Server für Pelletsheizung Fröling P2/S3100</h2>
          <ol>
            <li><a href=logfiles>Heizungs-Logfiles</a></li>
            <li><a href=syslogfile>System Logfiles</a></li>
            <li><a href=sysinfo>ESP32 System Info</a></li>
            <li><a href=esprestart>Restart ESP</a></li>
          </ol>
        <script>
        </script>
        </body>
        </html>
      )rawliteral";

  // Route for root / web page
  server.on(
      "/", HTTP_GET,
      [](AsyncWebServerRequest *request)
      { request->send_P(200, "text/html", index_html); });

  server.on(
      "/logfiles", HTTP_GET,
      [](AsyncWebServerRequest *request)
      { request->send(200, "text/html", handleLogfilesForWebserver(DATA_DIRECTORY).c_str()); });

  server.on(
      DATA_DIRECTORY, HTTP_GET, sendLargeFile); // replacement for serveStatic due to problems with larger files ...

  server.on(
      "/syslogfile", HTTP_GET,
      [](AsyncWebServerRequest *request)
      { request->send(200, "text/html", handleSysLogfilesForWebserver(SYSLOG_DIR).c_str()); });

  server.on(
      "/syslog", HTTP_GET, sendLargeFile); // replacement for serveStatic due to problems with larger files ...

  server.on(
      "/sysinfo", HTTP_GET,
      [](AsyncWebServerRequest *request)
      { request->send(200, "text/plain", get_System_Info().c_str()); });

  server.on(
      "/esprestart", HTTP_GET,
      [](AsyncWebServerRequest *request)
      {
        request->send(200, "text/plain", "Microcontroller ESP32 is restarted NOW ...");
        vTaskDelay(pdMS_TO_TICKS(1000)); // wait to deliver the webpage
        std::cout << millis() << " ms: ESP will be NOW restarted due to USER REQUEST from web interface";
        FILESYSTEM_TO_USE.end();
        ESP.restart();
      });

  server.serveStatic("/", FILESYSTEM_TO_USE, "/");

  server.onNotFound(
      [](AsyncWebServerRequest *request)
      { request->send(404, "text/plain", "404 - Not found"); });

  // before we start the webserver service we have to be sure that WiFi has finished initialisation
  while (WiFi.status() == WL_NO_SHIELD)
  {
    RADIATOR_LOG_DEBUG(millis() << " ms: Wait for WiFi to finished initialisation (WiFi.status() !=255 (WL_NO_SHIELD) ...) -> now WiFi.status()=" << WiFi.status() << std::endl;)
    vTaskDelay(500);
  }
  RADIATOR_LOG_DEBUG(millis() << " ms: Proceed with WiFi.status()=" << WiFi.status() << " -> start webserver" << std::endl;)

  // Start server
  server.begin();
  RADIATOR_LOG_INFO(millis() << " ms: Webserver started" << std::endl;)
}

/*********************************************************************
 * @brief 	handler for file request as replacement for server.serveStatic()
 *          due to problems of serveStatic for large files (> 20kB)
 * @param 	request
 * @return 	void
 *********************************************************************/
void radiator::NetworkHandler::sendLargeFile(AsyncWebServerRequest *request)
{
  fs::File file = FILESYSTEM_TO_USE.open(request->url().c_str());
  if (!file)
  {
    request->send(200, ((String) "ERROR opening file " + request->url()));
    // std::string message = (std::string) "ERROR opening file " + request->url().c_str();
    // request->send(200, message.c_str());
    return;
  }

  // std::cout << "file.size()=" << file.size() << std::endl;

  AsyncWebServerResponse *response = request->beginResponse(
      "text/plain; charset=utf-8",
      file.size(),
      [file](uint8_t *buffer, size_t maxLen, size_t total) mutable -> size_t
      {
        int bytes = file.read(buffer, maxLen);
        // std::cout << "maxLen=" << maxLen << ", total=" << total << ", bytes=" << bytes << std::endl;

        // close file at the end
        if (bytes + total == file.size())
          file.close();
        return max(0, bytes); // return 0 even when no bytes were loaded
      });

  // response->addHeader("meta", "charset=UTF-8");
  request->send(response);
}

/*********************************************************************
 * @brief 	list and handle ESP system logfiles for webserver
 * @param 	pathname for logfiles directory
 * @param 	used filesystem
 * @return 	text/html page with list of files in directory
 *          and download buttons (with javascript)
 *********************************************************************/
std::string radiator::NetworkHandler::handleSysLogfilesForWebserver(const char *dirname, fs::FS &fs)
{
  // static ulong nextLog = 0;
  // if (millis() >= nextLog)
  // {
  //   nextLog = millis() + 60000;
  //   DEBUG_STACK_HIGH_WATERMARK
  // }

  File root = fs.open(dirname);
  if (!root || !root.isDirectory())
  {
    RADIATOR_LOG_ERROR(millis() << " ms: handleSysLogfilesForWebserver: Failed to open directory " << dirname << std::endl;)
    return "";
  }

  bufStr = "";

  File file = root.openNextFile();
  while (file)
  {
    if (!file.isDirectory())
    {
      bufStr += "<li>"
                "<a href='" +
                (std::string)file.path() + "'>" + (std::string)file.name() + " (" + std::to_string(file.size()) +
                " bytes)   </a>"
                "<button type=button><a href='" +
                (std::string)file.path() +
                "' download> download</a></button>"
                "</li>";
    }
    file = root.openNextFile();
  }

  return "<!DOCTYPE HTML><html>"
         "<head>"
         "  <title>Pelletsheizung Froeling P2/S3100</title>"
         "  <meta charset='UTF-8'>"
         "  <meta name='viewport' content='width=device-width, initial-scale=1.0'>"
         "  <link rel='icon' href='/favicon.ico' 'type='image/x-icon'>"
         "</head>"
         "<body>"
         "  <h2>ESP Web Server für Pelletsheizung Fröling P2/S3100</h2>"
         "  <ul>" +
         bufStr +
         "  </ul>"
         "</body></html>";
}

/*********************************************************************
 * @brief 	list and handle radiator logfiles for webserver
 * @param 	pathname for logfiles directory
 * @param 	used filesystem
 * @return 	text/html page with list of files in directory
 *          and download buttons (with javascript)
 *********************************************************************/
std::string radiator::NetworkHandler::handleLogfilesForWebserver(const char *dirname, fs::FS &fs)
{
  File root = fs.open(dirname);
  if (!root || !root.isDirectory())
  {
    RADIATOR_LOG_ERROR(millis() << " ms: handleLogfilesForWebserver: Failed to open directory " << dirname << std::endl;)
    return "";
  }

  bufStr = "";

  File file = root.openNextFile();
  while (file)
  {
    if (!file.isDirectory())
    {
      bufStr += "<li>"
                "<a href='" +
                (std::string)file.path() + "'>" + (std::string)file.name() + " (" + std::to_string(file.size()) +
                " bytes)   </a>"
                "<button type=button><a href='" +
                (std::string)file.path() +
                "' download> download</a></button>"
                "</li>";
    }
    file = root.openNextFile();
  }

  return "<!DOCTYPE HTML><html>"
         " <head>"
         "  <title>Pelletsheizung Froeling P2/S3100</title>"
         "  <meta charset='UTF-8'>"
         "  <meta name='viewport' content='width=device-width, initial-scale=1.0'>"
         "  <link rel='icon' href='/favicon.ico' 'type='image/x-icon'>"
         "</head>"
         "<body>"
         "  <h2>ESP Web Server für Pelletsheizung Fröling P2/S3100</h2>"
         "  <ul>" +
         bufStr +
         "  </ul>"
         "</body></html>";
}
#endif // #if START_WEBSERVER