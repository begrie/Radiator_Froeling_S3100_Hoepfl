#ifndef __DH_NETWORK_H__
#define __DH_NETWORK_H__

#include <WiFi.h>

#if START_WEBSERVER
#include <ESPAsyncWebServer.h>
#endif

#include <AsyncMqttClient.h>
#include <Ticker.h>
// #include <chrono>

#include "config.h"

namespace radiator
{
  class NetworkHandler
  {
  public:
    static bool init(const bool _outputToMQTT = false, const bool startWebServer = true);
    static bool publishToMQTT(const std::string &payload, const std::string &subtopic = "", const uint8_t qos = 1, const bool retain = true);

  protected:
    static bool configureWiFiAndMQTT();
    static bool inputMQTTconfig();
    static void installWiFiCallbacks();

    static void loadMQTTConfigFromPreferences();
    static void configureMQTT();
    static std::string printMQTTConfig();
    static void onMqttConnect(bool sessionPresent);
    static void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
    static void onMqttSubscribe(uint16_t packetId, uint8_t qos);
    static void onMqttUnsubscribe(uint16_t packetId);
    static void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total);
    static void onMqttPublish(uint16_t packetId);

    static void connectToMqtt();
    static void disconnectMqtt();

    static AsyncMqttClient mqttClient;
    static bool outputToMQTT;
    static Ticker tickerReconnectMQTT;
    static Ticker tickerSendSysinfoToMQTT;
    static std::string mqttTopic;
    static std::string mqttBroker;
    static std::string mqttUser;
    static std::string mqttPassword;

    static std::string get_System_Info();

#if START_WEBSERVER
    static void configureWebserver();
    static void sendLargeFile(AsyncWebServerRequest *request);
    static std::string handleSysLogfilesForWebserver(const char *dirname, fs::FS &fs = FILESYSTEM_TO_USE);
    static std::string handleLogfilesForWebserver(const char *dirname, fs::FS &fs = FILESYSTEM_TO_USE);
#endif
    static std::string bufStr;              // as class member to avoid heap fragmentation
    static std::ostringstream bufStrStream; // as class member to avoid heap fragmentation
  };
}
#endif // #ifndef __DH_NETWORK_H__