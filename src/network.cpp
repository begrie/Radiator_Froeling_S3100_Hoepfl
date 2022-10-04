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
std::string radiator::NetworkHandler::mqttTopic = MQTT_TOPIC;
std::string radiator::NetworkHandler::mqttBroker = MQTT_BROKER;

/*********************************************************************
 * @brief 	Init NetworkHandler
 * @param 	true: enable output to MQTT
 *          false: disable output to MQTT
 * @param 	true: enable webserver
 *          false: disable webserver
 * @return 	void
 *********************************************************************/
bool radiator::NetworkHandler::init(bool _outputToMQTT, bool startWebServer)
{
  outputToMQTT = _outputToMQTT;

  installWiFiCallbacks();

  if (outputToMQTT)
    configureMQTT();

  if (!configureWiFiAndMQTT())
    return false;

  if (startWebServer)
    configureWebserver();

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
  const auto quitButtonPin = QUIT_BUZZER_BUTTON_PIN;
  pinMode(quitButtonPin, INPUT_PULLUP);

  bool newWiFiConfig = false;

  if (!digitalRead(quitButtonPin)) // LOW -> button is pressed for new configuration
  {
    std::cout << "\n\nWiFi: Button pressed at startup for WiFi and MQTT configuration:\n"
              << "\tNow scanning for WiFi networks ..."
              << std::endl;

    auto numberOfFoundNetworks = WiFi.scanNetworks(false); // scan in blocking mode

    std::cout << "Scan done -> found " << numberOfFoundNetworks << " WiFi networks:" << std::endl;

    for (int i = 0; i < numberOfFoundNetworks; i++)
    {
      std::cout
          << "[" << i + 1 << "] " << WiFi.SSID(i).c_str()
          << "\t RSSI=" << WiFi.RSSI(i)
          << std::endl;
    }

    auto oldTimeout = Serial.getTimeout();
    Serial.setTimeout(ULONG_MAX);

    int intInput;
    std::string strInput;
    bool inputOk = false;

    do
    {
      std::cout << "\t-> Select network by [number] \n\t-> or 0 to turn WiFi off \n\t-> or -1 to skip and proceed with MQTT (actual WiFi settings will be used):    " << std::endl;
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
      return false;
    }

    if (intInput != -1)
    {
      std::cout << "Please input password for WLAN with SSID  " << WiFi.SSID(intInput).c_str() << ":" << std::endl;
      strInput = Serial.readStringUntil('\r').c_str();

      std::cout << "SSID and password are stored persistent for automatic WiFi connection at startup" << std::endl;
      WiFi.persistent(true);
      newWiFiConfig = true;
    }

    inputMQTTconfig();

    if (newWiFiConfig)
      WiFi.begin(WiFi.SSID(intInput - 1).c_str(), strInput.c_str());

    Serial.setTimeout(oldTimeout); // restore previous value
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
 * @return 	void
 *********************************************************************/
void radiator::NetworkHandler::inputMQTTconfig()
{
  auto toFind = std::regex("([a-zA-Z0-9-.]{2,256}\\.[a-z]{2,4})"); // finds url in string e.g.   broker.hivemq.com
                                                                   // https://regexr.com
  std::smatch result;
  std::string strInput;

  std::cout << printMQTTConfig() << std::endl;
  do
  {
    std::cout << "Set MQTT broker (ENTER to use actual) hostname: " << std::endl;

    strInput = Serial.readStringUntil('\r').c_str();
    auto res = std::regex_search(strInput, result, toFind);
    LOG_debug << "strInput=" << strInput << ", regex_search->result=" << result.str() << std::endl;

  } while (result.empty() && strInput.size() > 1);

  if (!result.empty())
  {
    mqttBroker = result.str();

    Preferences mqttPrefs;
    mqttPrefs.begin(PREFERENCES_NAMESPACE);
    mqttPrefs.putString("mqttBroker", mqttBroker.c_str());
    mqttPrefs.end();

    LOG_info << "Saved mqttBroker hostname to preferences in NVS" << std::endl;

    configureMQTT();
  }
}

/*********************************************************************
 * @brief 	install needed WiFi callbacks as Lambda's
 * @param 	void
 * @return 	void
 *********************************************************************/
void radiator::NetworkHandler::installWiFiCallbacks()
{
  WiFi.onEvent(
      [](WiFiEvent_t _WiFi_Event, WiFiEventInfo_t _WiFi_Event_Info)
      {
        LOG_info << "\nWiFi connected to... \n"
                 << "\t WLAN / SSID: \t" << WiFi.SSID().c_str() << "\n"
                 << "\t Signal: \tRSSI " << std::to_string(WiFi.RSSI()) << "\n"
                 << "\t Hostname: \t" << WiFi.getHostname() << "\n"
                 << "\t IP address: \t" << WiFi.localIP().toString().c_str() << "\n"
                 << "\t Gateway IP: \t" << WiFi.gatewayIP().toString().c_str() << "\n"
                 << "\t Network IP: \t" << WiFi.networkID().toString().c_str() << "\n"
                 << std::endl;

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

  WiFi.onEvent(
      [](WiFiEvent_t _WiFi_Event, WiFiEventInfo_t _WiFi_Event_Info)
      {
        static ulong timeConnectionLostMs = 0;
        if (WiFi.status() == WL_CONNECTION_LOST)
        {
          LOG_info
              << "\nWiFi was DISCONNECTED from WLAN. System is trying to reconnect in a background task ... \n"
              << "(Info: Change WiFi or MQTT settings by pressing the big yellow button at startup of the ESP32)\n"
              << std::endl;
          timeConnectionLostMs = millis();

          // disconnectMqtt();
        }

        static ulong nextInfoOutputMs = 0;
        static const int infoOutputIntervallSec = 10 * 60; // info output all 10 minutes -> consider needed space when output is redirected to syslog file

        if (WiFi.status() == WL_NO_SSID_AVAIL && millis() >= nextInfoOutputMs)
        {
          LOG_info
              << "\nWiFi is NOT CONNECTED to WLAN -> SSID not available. The system keeps trying to reconnect in a background task ... \n"
              << "\t(Connection was lost " << ((millis() - timeConnectionLostMs) / (1000 * 60)) << " minutes ago.)\n"
              << "\t(Info: Change WiFi or MQTT settings by pressing the big yellow button at startup of the ESP32)\n"
              << std::endl;
          nextInfoOutputMs = millis() + infoOutputIntervallSec * 1000;

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

  WiFi.onEvent(
      [](WiFiEvent_t _WiFi_Event, WiFiEventInfo_t _WiFi_Event_Info)
      {
        LOG_info << "\nWiFi:\tGot IP address:\t" << WiFi.localIP().toString().c_str() << "\n"
                 << "\tGateway IP: \t" << WiFi.gatewayIP().toString().c_str() << "\n"
                 << "\tNetwork IP: \t" << WiFi.networkID().toString().c_str() << "\n"
                 << std::endl;

        if (outputToMQTT)
          connectToMqtt();
      },
      WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
}

/*********************************************************************
 * @brief 	configure MQTT AsyncMqttClient
 * @param 	void
 * @return 	void
 *********************************************************************/
void radiator::NetworkHandler::configureMQTT()
{
  // load from preferences-flash
  Preferences mqttPrefs;
  mqttPrefs.begin(PREFERENCES_NAMESPACE);
  mqttBroker = mqttPrefs.getString("mqttBroker", MQTT_BROKER).c_str();
  mqttPrefs.end();

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);

  mqttClient.setServer(mqttBroker.c_str(), MQTT_PORT);
  mqttClient.setKeepAlive(MQTT_KEEP_ALIVE);
  mqttClient.setCleanSession(false); // hold queued messages after disconnect -> consider needed ram at longer disconnection intervals

  static std::string lastWillTopic = mqttTopic + "/onlinestatus";
  static std::string lastWill = (std::string)mqttClient.getClientId() + ": offline";
  mqttClient.setWill(lastWillTopic.c_str(), 1, true, lastWill.c_str()); // topic and payload for lastWill must be defined static!!

  std::cout << printMQTTConfig() << std::endl;
}

/*********************************************************************
 * @brief 	print config info about mqtt client
 * @param 	void
 * @return 	formatted infos as string
 *********************************************************************/
std::string radiator::NetworkHandler::printMQTTConfig()
{
  return "MQTT config: \t Broker= " + mqttBroker + ", Port= " + std::to_string(MQTT_PORT) + ", KeepAlive= " + std::to_string((int)MQTT_KEEP_ALIVE) + " sec \n" + "\t\t ClientID= " + mqttClient.getClientId() + ", Topic= " + mqttTopic;
}

/*********************************************************************
 * @brief 	callback for MQTT connection
 * @param 	session present
 * @return 	void
 *********************************************************************/
void radiator::NetworkHandler::onMqttConnect(bool sessionPresent)
{
  LOG_info << millis() << " ms: Connected to MQTT: Session present= " << sessionPresent
           << ", mqttClient.getClientId()=" << mqttClient.getClientId() << std::endl;

  std::string statusInfo = (std::string)mqttClient.getClientId() + ": online";
  mqttClient.publish((mqttTopic + "/onlinestatus").c_str(), 1, true, statusInfo.c_str());
  mqttClient.publish((mqttTopic + "/systeminfo").c_str(), 1, true, get_System_Info().c_str());
}

/*********************************************************************
 * @brief 	callback for MQTT disconnection
 * @param 	reason for disconnection
 * @return 	void
 *********************************************************************/
void radiator::NetworkHandler::onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  std::string reasonStr = "UNKNOWN";

  switch (reason)
  {
  case AsyncMqttClientDisconnectReason::TCP_DISCONNECTED:
    reasonStr = "TCP_DISCONNECTED";
    break;
  case AsyncMqttClientDisconnectReason::MQTT_UNACCEPTABLE_PROTOCOL_VERSION:
    reasonStr = "MQTT_UNACCEPTABLE_PROTOCOL_VERSION";
    break;
  case AsyncMqttClientDisconnectReason::MQTT_IDENTIFIER_REJECTED:
    reasonStr = "MQTT_IDENTIFIER_REJECTED";
    break;
  case AsyncMqttClientDisconnectReason::MQTT_SERVER_UNAVAILABLE:
    reasonStr = "MQTT_SERVER_UNAVAILABLE";
    break;
  case AsyncMqttClientDisconnectReason::MQTT_MALFORMED_CREDENTIALS:
    reasonStr = "MQTT_MALFORMED_CREDENTIALS";
    break;
  case AsyncMqttClientDisconnectReason::MQTT_NOT_AUTHORIZED:
    reasonStr = "MQTT_NOT_AUTHORIZED";
    break;
  case AsyncMqttClientDisconnectReason::ESP8266_NOT_ENOUGH_SPACE:
    reasonStr = "ESP8266_NOT_ENOUGH_SPACE";
    break;
  case AsyncMqttClientDisconnectReason::TLS_BAD_FINGERPRINT:
    reasonStr = "TLS_BAD_FINGERPRINT";
    break;
  }

  LOG_info << millis() << " ms: Disconnected from MQTT: reason = " << reasonStr << std::endl;

  if (WiFi.isConnected() && outputToMQTT)
  {
    LOG_info << millis() << " ms: Start ticker for reconnect to MQTT broker in " << MQTT_RECONNECTION_TIMEOUT_SEC << " sec." << std::endl;
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
  LOG_info << millis() << " ms: Subscribe acknowledged: packetId= " << packetId << ", qos= " << (int)qos << std::endl;
}

/*********************************************************************
 * @brief 	callback for MQTT topic UNsubscription
 * @param 	packet ID
 * @return 	void
 *********************************************************************/
void radiator::NetworkHandler::onMqttUnsubscribe(uint16_t packetId)
{
  LOG_info << millis() << " ms: Unsubscribe acknowledged: packetId= " << packetId << std::endl;
}

/*********************************************************************
 * @brief 	callback for received MQTT message
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
  LOG_info << millis()
           << " ms: Published message received: \n"
           << "\t topic= " << topic << "\n"
           << "\t payload= " << payload << "\n"
           << "\t qos= " << (int)properties.qos << "\n"
           << "\t dup= " << properties.dup << "\n"
           << "\t retain= " << properties.retain << "\n"
           << "\t len= " << len << "\n"
           << "\t index= " << index << "\n"
           << "\t total= " << total << std::endl;
}

/*********************************************************************
 * @brief 	callback for MQTT publish from client
 * @param 	packet ID
 * @return 	void
 *********************************************************************/
void radiator::NetworkHandler::onMqttPublish(uint16_t packetId)
{
  LOG_info << millis() << " ms: Publish acknowledged for packetId= " << packetId << std::endl;
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
    LOG_info << millis() << " ms: Output to MQTT is disabled by config (outputToMQTT=false)" << std::endl;
    return;
  }

  if (mqttClient.connected())
  {
    LOG_info << millis() << " ms: MQTT client is already connected" << std::endl;
    return;
  }

  // mqttClient.clearQueue();

  LOG_info << millis() << " ms: Connecting to MQTT broker    " << mqttBroker << std::endl;
  mqttClient.connect();
}

/*********************************************************************
 * @brief 	disconnects MQTT from broker
 * @param 	void
 * @return 	void
 *********************************************************************/
void radiator::NetworkHandler::disconnectMqtt()
{
  LOG_info << millis() << " ms: DISconnecting MQTT: client ID=" << mqttClient.getClientId() << std::endl;

  if (!mqttClient.connected())
  {
    LOG_info << millis() << " ms: ... MQTT client is NOT connected" << std::endl;
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
bool radiator::NetworkHandler::publishToMQTT(std::string payload, std::string subtopic, uint8_t qos, bool retain)
{
  if (!outputToMQTT)
  {
    LOG_warn << millis() << " ms: Output to MQTT is disabled by config" << std::endl;
    return false;
  }

  // mqttClient itself buffers only until disconnection from MQTT broker - so we have to buffer and resend the data on our own
  struct BuffStruct
  {
    std::string payload;
    std::string subtopic;
    uint8_t qos;
    bool retain;
  };
  static std::deque<BuffStruct> bufferQueue;

  while (ESP.getFreeHeap() < MIN_FREE_HEAPSIZE_FOR_MQTT_BUFFERQUEUE_BYTES)
  {
    LOG_warn << "Old MQTT messages cannot be buffered any more due to low free heap size: ESP.getFreeHeap()="
             << ESP.getFreeHeap() << " bytes -> oldest buffered message will be deleted (bufferQueue.size="
             << bufferQueue.size() << " elements)" << std::endl;

    if (!bufferQueue.empty())
      bufferQueue.pop_front();
  }

  // ALL messages are pushed to the buffer queue
  LOG_debug << "1: bufferQueue.size=" << bufferQueue.size() << " ESP.getFreeHeap()=" << (ESP.getFreeHeap()) << " bytes" << std::endl;
  bufferQueue.push_back({payload, subtopic, qos, retain});
  LOG_debug << "2: bufferQueue.size=" << bufferQueue.size() << " ESP.getFreeHeap()=" << (ESP.getFreeHeap()) << " bytes" << std::endl;

  if (!mqttClient.connected())
  {
    LOG_warn << millis() << " ms: mqttClient is not connected! Cannot publish to MQTT broker (" << mqttBroker << ")\n"
             << "\t -> message is buffered (queue size=" << bufferQueue.size() << ", ESP.getFreeHeap()=" << (ESP.getFreeHeap())
             << ") and send at reconnection \n"
             << "\t topic = " << (mqttTopic + bufferQueue.back().subtopic) << "\n"
             << "\t payload = " << bufferQueue.back().payload << std::endl;
    return false;
  }

  uint16_t packetId;

  while (!bufferQueue.empty())
  {
    std::string topic = mqttTopic + bufferQueue.front().subtopic;

    packetId = mqttClient.publish(topic.c_str(),
                                  bufferQueue.front().qos,
                                  bufferQueue.front().retain,
                                  bufferQueue.front().payload.c_str());

    LOG_info << millis() << " ms: publishToMQTT: broker = " << mqttBroker
             << ", topic = " << topic
             << ", Quality of service = " << (int)bufferQueue.front().qos
             << ", retain = " << bufferQueue.front().retain << ", packetId = " << packetId
             << ", \n\tpayload=\n\t" << bufferQueue.front().payload << std::endl;

    bufferQueue.pop_front(); // remove sent message from buffer queue
  }

  return packetId; // 0 on failure
}

/*********************************************************************
 * @brief 	assemble and format system info to a std::string
 * @param   void
 * @return 	systeminfo
 *********************************************************************/
std::string radiator::NetworkHandler::get_System_Info()
{
  std::stringstream systemInfo;

  systemInfo << "Microcontroller ESP32: \n";
  systemInfo << "ESP32 chip model  : " << ESP.getChipModel() << "\n";
  systemInfo << "Chip Revision     : " << ((int)ESP.getChipRevision()) << "\n";
  systemInfo << "CPU Frequency     : " << ESP.getCpuFreqMHz() << " MHz \n";
  systemInfo << "Chip cores        : " << ((int)ESP.getChipCores()) << "\n";
  systemInfo << "Flash chip size   : " << (ESP.getFlashChipSize() / 1024) << " kBytes \n";
  systemInfo << "Flash chip speed  : " << (ESP.getFlashChipSpeed() / 1000000) << " MHz \n";
  systemInfo << "Sketch size       : " << (ESP.getSketchSize() / 1024) << " kBytes \n";
  systemInfo << "Free sketch space : " << (ESP.getFreeSketchSpace() / 1024) << " kBytes \n";
  systemInfo << "SDK version       : " << ESP.getSdkVersion() << "\n";

  systemInfo << "\n";

  systemInfo << "Uptime            : " << (millis() / (1000 * 60)) << " min \n";
  systemInfo << "Heapsize          : " << (ESP.getHeapSize() / 1024) << " kBytes \n";
  systemInfo << "Free heap         : " << (ESP.getFreeHeap() / 1024) << " kBytes \n";
  systemInfo << "Min. free heap    : " << (ESP.getMinFreeHeap() / 1024) << " kBytes \n";
  systemInfo << "Max. alloc heap   : " << (ESP.getMaxAllocHeap() / 1024) << " kBytes \n";
  systemInfo << "Free PSRAM        : " << ESP.getFreePsram() << " Bytes \n";
  systemInfo << "Filesystem        : " << XSTRINGIFY(FILESYSTEM_TO_USE) << " \n";
  systemInfo << "Basepath(Mountpt.): " << FILESYSTEM_BASE_PATH << " \n";
  systemInfo << "Used Filesystem   : " << (FILESYSTEM_TO_USE.usedBytes() / 1024) << " kBytes \n";
  systemInfo << "Free Filesystem   : " << ((FILESYSTEM_TO_USE.totalBytes() - SPIFFS.usedBytes()) / 1024) << " kBytes \n";

  systemInfo << "\n";

  systemInfo << "WiFi IP            : " << WiFi.localIP().toString().c_str() << "\n";
  systemInfo << "WiFi hostname      : " << WiFi.getHostname() << "\n";
  systemInfo << "WiFi SSID          : " << WiFi.SSID().c_str() << "\n";
  systemInfo << "WiFi status        : " << WiFi.status() << "\n";
  systemInfo << "WiFi strength RSSI : " << ((int)WiFi.RSSI()) << " dB\n";
  systemInfo << "WiFi MAC           : " << WiFi.macAddress().c_str() << "\n";
  systemInfo << "WiFi subnet        : " << WiFi.subnetMask().toString().c_str() << "\n";
  systemInfo << "WiFi gateway       : " << WiFi.gatewayIP().toString().c_str() << "\n";
  systemInfo << "WiFi DNS 1         : " << WiFi.dnsIP(0).toString().c_str() << "\n";
  systemInfo << "WiFi DNS 2         : " << WiFi.dnsIP(1).toString().c_str() << "\n";

  systemInfo << std::endl;
  // std::cout << systemInfo.str();
  return systemInfo.str();
}

/*********************************************************************
 * @brief 	configure Webserver
 * @param 	void
 * @return 	void
 *********************************************************************/
void radiator::NetworkHandler::configureWebserver()
{
  static AsyncWebServer server(80); // Create AsyncWebServer object on port 80

  // create very simple html page
  static const char index_html[] PROGMEM =
      R"rawliteral(
        <!DOCTYPE HTML><html>
        <head>
          <title>Pelletsheizung Froeling P2/S3100</title>
          <meta charset="UTF-8">
          <meta name="viewport" content="width=device-width, initial-scale=1.0">
          <link rel="icon" href="data:,">
          <style>
          </style>
        </head>
        <body>
          <h2>ESP Web Server für Pelletsheizung Fröling P2/S3100</h2>
          <ul>
            <li><a href=logfiles>Heizungs-Logfiles</a></li>
            <li><a href=sysinfo>ESP32 System Info</a></li>
            <li><a href=syslogfile>System Logfile</a></li>
          </ul>
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

  server.serveStatic("/", FILESYSTEM_TO_USE, DATA_DIRECTORY);

  server.onNotFound(
      [](AsyncWebServerRequest *request)
      { request->send(404, "text/plain", "404 - Not found"); });

  server.on(
      "/logfiles", HTTP_GET, [](AsyncWebServerRequest *request)
      { request->send(200, "text/html", handleLogfilesForWebserver(DATA_DIRECTORY).c_str()); });

  server.on(
      "/downloadAllAsOneFile", HTTP_GET, [](AsyncWebServerRequest *request)
      { request->send(200, "text/html", downloadAllAsOneFile(DATA_DIRECTORY).c_str()); });

  server.on(
      "/sysinfo", HTTP_GET, [](AsyncWebServerRequest *request)
      { request->send(200, "text/plain", get_System_Info().c_str()); });

  server.serveStatic("/syslogfile", FILESYSTEM_TO_USE, SYSLOG_FILENAME);

  // before we start the webserver service we have to be sure that WiFi has finished initialisation
  while (WiFi.status() == WL_NO_SHIELD)
  {
    LOG_info << millis() << " ms: Wait for WiFi to finished initialisation (WiFi.status() !=255 (WL_NO_SHIELD) ...) -> now WiFi.status()="
             << WiFi.status() << std::endl;
    vTaskDelay(500);
  }
  LOG_info << millis() << " ms: Proceed with WiFi.status()=" << WiFi.status() << " -> start webserver" << std::endl;

  // Start server
  server.begin();
  LOG_info << millis() << " ms: Webserver started" << std::endl;
}

/*********************************************************************
 * @brief 	list and handle radiator logfiles for webserver
 * @param 	pathname for logfiles directory
 * @param 	used filesystem
 * @return 	text/html page with list of files in directory
 *          and download buttons (with javascript)
 *********************************************************************/
std::string radiator::NetworkHandler::handleLogfilesForWebserver(std::string dirname, fs::FS &fs)
{
  File root = fs.open(dirname.c_str());
  if (!root || !root.isDirectory())
  {
    LOG_error << "Failed to open directory " << dirname << std::endl;
    return "";
  }

  std::stringstream filesDownloadList;
  std::stringstream filesUrlsList;

  File file = root.openNextFile();
  while (file)
  {
    if (!file.isDirectory())
    {
      filesUrlsList << "<li>"
                    << "<a href='" << file.name() << "'>"
                    << file.name() << " (" << file.size() << " bytes)   </a>"
                    << "<button type=button><a href='" << file.name() << "' download> download</a></button>"
                    << "</li>";
      filesDownloadList << "{ download: '" << file.name() << "' }";
    }
    file = root.openNextFile();
    if (file)
      filesDownloadList << ",";
  }

  std::stringstream logfilesPage;
  logfilesPage << "<!DOCTYPE HTML><html>"
               << " <head>"
               << "  <title>Pelletsheizung Froeling P2/S3100</title>"
               << "  <meta charset='UTF-8'>"
               << "  <meta name='viewport' content='width=device-width, initial-scale=1.0'>"
               << "  <link rel='icon' href='data:,'>"
               << "</head>"
               << "<script> "
               << "  function downloadAllFiles(files)"
               << "  {"
               << "      files.forEach(function(el)"
               << "      {"
               << "        let a = document.createElement('a');"
               << "        a.href = el.download;"
               << "        a.target = '_parent';"
               << "        a.download = el.filename;"
               << "        (document.body || document.documentElement).appendChild(a);"
               << "        a.click();"
               << "        a.parentNode.removeChild(a);"
               << "      });"
               << "  }"
               << "</script>"
               << "<body>"
               << "  <h2>ESP Web Server für Pelletsheizung Fröling P2/S3100</h2>"
               << "  <p>"
               << "  <button type=button onclick=\"downloadAllFiles([" << filesDownloadList.str() << "])\">Download ALL files</button>"
               << "  </p><p>"
               << "  <button type=button><a href=downloadAllAsOneFile>Create ONE FILE FROM LAST 3 MONTH DATA for download </br>!!! THIS WILL LAST A WHILE !!!</a></button>"
               << "  </p>"
               << "  <ul>"
               << filesUrlsList.str()
               << "  </ul>"
               << "</body></html>"
               << std::endl;

  return logfilesPage.str();
}

/*********************************************************************
 * @brief 	download NUMBER_OF_LOGFILES_TO_COMBINE logfiles as ONE FILE
 * @param 	source dir with logfiles to combine
 * @param 	used filesystem
 * @return 	link to file for download
 *********************************************************************/
std::string radiator::NetworkHandler::downloadAllAsOneFile(std::string dirname, fs::FS &fs)
{
  File root = fs.open(dirname.c_str());
  if (!root || !root.isDirectory())
  {
    auto buf = "Failed to open directory " + dirname;
    LOG_error << buf << std::endl;
    return buf;
  }

  // create list from filenames for sorting and selecting timespan
  std::list<std::string> filesList;
  File file = root.openNextFile();
  while (file)
  {
    if (!file.isDirectory())
      filesList.push_back(file.path());

    file = root.openNextFile();
  }
  filesList.sort();

  // save only last 3 month = 92 days = no. of files
  while (filesList.size() > NUMBER_OF_LOGFILES_TO_COMBINE)
  {
    filesList.pop_front();
  }

  // LOG_info << "Sorted files list: \n";
  // for (auto el : filesList)
  //   LOG_info << el << "\n";

  std::string subdirName = "/union";
  fs.mkdir((dirname + subdirName).c_str());
  std::string targetFilename = "/last3Month.log";
  std::string targetPathname = dirname + subdirName + targetFilename;
  fs.remove(targetPathname.c_str()); // we need an empty file
  File targetFile = fs.open(targetPathname.c_str(), FILE_APPEND);
  if (!targetFile)
  {
    auto buf = "Failed to open target file  " + targetPathname + "  for writing";
    LOG_error << buf << std::endl;
    return buf;
  }

  for (auto el : filesList)
  {
    fs::File sourceFile = fs.open(el.c_str(), FILE_READ);
    if (!sourceFile.isDirectory() && strcmp(sourceFile.path(), targetFile.path()) != 0)
    {
      size_t written_bytes = 0;
      while (sourceFile.available())
      {
        written_bytes += targetFile.write(sourceFile.read());
        // std::cout << '.';
      }
      LOG_info << "  Written " << written_bytes << " bytes from " << sourceFile.name() << " to " << targetFile.name() << std::endl;
    }
    sourceFile.close();
  }
  targetFile.flush();

  std::stringstream page;
  page << "<!DOCTYPE HTML><html>"
       << " <head>"
       << "  <title>Pelletsheizung Froeling P2/S3100</title>"
       << "  <meta charset='UTF-8'>"
       << "  <meta name='viewport' content='width=device-width, initial-scale=1.0'>"
       << "  <link rel='icon' href='data:,'>"
       << "</head>"
       << "<body>"
       << "  <h2>ESP Web Server für Pelletsheizung Fröling P2/S3100</h2>"
       << "  <a href='" << (subdirName + targetFilename) << "'>" << (subdirName + targetFilename) << " (" << targetFile.size() << " bytes)   </a>"
       << "  <button type=button><a href='" << (subdirName + targetFilename) << "' download> download</a></button>"
       << "  </br></br>"
       << "  <a href=/>Home</a>"
       << "</body></html>"
       << std::endl;

  return page.str();
}
