#ifndef __DH_NETWORK_H__
#define __DH_NETWORK_H__

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncMqttClient.h>
#include <Ticker.h>

#include "config.h"

namespace radiator
{
  class NetworkHandler
  {
  protected:
    static bool configureWiFiAndMQTT();
    static void inputMQTTconfig();
    static void installWiFiCallbacks();

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

    static std::string get_System_Info();

    static void configureWebserver();
    static void sendLargeFile(AsyncWebServerRequest *request);
    static std::string handleSysLogfilesForWebserver(std::string dirname, fs::FS &fs = FILESYSTEM_TO_USE);
    static std::string handleLogfilesForWebserver(std::string dirname, fs::FS &fs = FILESYSTEM_TO_USE);
    static std::string downloadLogfilesAsOne(std::string dirname, uint16_t NumberOfLogfiles = NUMBER_OF_LOGFILES_TO_COMBINE, fs::FS &fs = FILESYSTEM_TO_USE);

  public:
    static bool init(bool _outputToMQTT = false, bool startWebServer = true);
    static bool publishToMQTT(std::string payload, std::string subtopic = "", uint8_t qos = 1, bool retain = true);
  };
}
#endif //#ifndef __DH_NETWORK_H__