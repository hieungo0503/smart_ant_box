#ifndef THINGSBOARD_CLIENT_H
#define THINGSBOARD_CLIENT_H

#include <WiFi.h>
#include <config.h>
#include <ThingsBoard.h>
#include <Server_Side_RPC.h>
#include <Arduino_MQTT_Client.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// Forward declarations
class PIDController;
class TemperatureSensor;
class WiFiManager;

class ThingsBoardClient
{
public:
    // Constructor
    ThingsBoardClient();

    // Destructor
    ~ThingsBoardClient();

    // Initialize ThingsBoard connection
    bool begin();

    // Start the ThingsBoard communication task
    bool startTask(SemaphoreHandle_t dataMutex, PIDController *pidController, TemperatureSensor *tempSensor, WiFiManager *wifiMgr);

    // Stop the ThingsBoard communication task
    void stopTask();

    // Check if connected to ThingsBoard
    bool isConnected();

    // Send telemetry data manually (optional - normally done automatically)
    bool sendTelemetryData(const char *key, double value);
    bool sendTelemetryData(const char *key, int value);
    bool sendTelemetryData(const char *key, bool value);

private:
    // WiFi and ThingsBoard objects
    WiFiClient wifiClient;
    Arduino_MQTT_Client *mqttClient;
    Server_Side_RPC<MAX_RPC_SUBSCRIPTIONS, MAX_RPC_RESPONSE> *rpc;
    ThingsBoard *tb;

    // Task handle
    TaskHandle_t taskHandle;

    // Mutex for data protection
    SemaphoreHandle_t dataMutex;

    // References to other components
    PIDController *pidController;
    TemperatureSensor *temperatureSensor;
    WiFiManager *wifiManager;

    // Connection status
    bool subscribed;
    unsigned long lastTelemetrySend;

    // Task function (static)
    static void thingsBoardTask(void *pvParameters);

    // Internal methods
    bool connectToThingsBoard();
    bool subscribeToRPC();
    void sendTelemetryLoop();
    void processThingsBoardLoop();

    // RPC callback functions (static)
    static void processSetTargetTemp(const JsonVariantConst &data, JsonDocument &response);
    static void processSetPidParams(const JsonVariantConst &data, JsonDocument &response);

    // Static reference to current instance for RPC callbacks
    static ThingsBoardClient *instance;

    // Helper methods for RPC
    void handleSetTargetTemp(const JsonVariantConst &data, JsonDocument &response);
    void handleSetPidParams(const JsonVariantConst &data, JsonDocument &response);
};

#endif // THINGSBOARD_CLIENT_H