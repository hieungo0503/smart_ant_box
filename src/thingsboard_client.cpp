#include "thingsboard_client.h"
#include "config.h"
#include "pid_controller.h"
#include "temperature_sensor.h"
#include "wifi_manager.h"
#include <Arduino.h>

// Static instance reference for RPC callbacks
ThingsBoardClient *ThingsBoardClient::instance = nullptr;

ThingsBoardClient::ThingsBoardClient()
    : mqttClient(nullptr), rpc(nullptr), tb(nullptr), taskHandle(NULL),
      dataMutex(NULL), pidController(nullptr), temperatureSensor(nullptr),
      subscribed(false), lastTelemetrySend(0)
{

    // Set static instance reference
    instance = this;

    // Initialize MQTT client
    mqttClient = new Arduino_MQTT_Client(wifiClient);

    // Initialize RPC handler
    rpc = new Server_Side_RPC<MAX_RPC_SUBSCRIPTIONS, MAX_RPC_RESPONSE>();

    // Initialize ThingsBoard client without API implementations first
    tb = new ThingsBoard(*mqttClient, MAX_MESSAGE_RECEIVE_SIZE, MAX_MESSAGE_SEND_SIZE,
                         Default_Max_Stack_Size);

    // Subscribe the RPC API implementation after construction
    tb->Subscribe_API_Implementation(*rpc);
}

ThingsBoardClient::~ThingsBoardClient()
{
    stopTask();
    delete tb;
    delete rpc;
    delete mqttClient;
    instance = nullptr;
    preferences.end();
}

bool ThingsBoardClient::begin()
{
    // Initialize Preferences for storing configuration
    preferences.begin("smart_config", false);

    Serial.println("ThingsBoard Client: Initializing...");

    if (tb == nullptr || rpc == nullptr || mqttClient == nullptr)
    {
        Serial.println("ThingsBoard Client: Failed to create ThingsBoard objects");
        return false;
    }

    Serial.println("ThingsBoard Client: Initialization complete");
    return true;
}

bool ThingsBoardClient::startTask(SemaphoreHandle_t mutex, PIDController *pid, TemperatureSensor *temp, WiFiManager *wifiMgr)
{
    if (taskHandle != NULL)
    {
        Serial.println("ThingsBoard Client: Task already running");
        return false;
    }

    dataMutex = mutex;
    pidController = pid;
    temperatureSensor = temp;
    wifiManager = wifiMgr;

    BaseType_t result = xTaskCreatePinnedToCore(
        thingsBoardTask,
        "ThingsBoardTask",
        8192,
        this, // Pass this instance as parameter
        1,    // Lower priority for ThingsBoard communication
        &taskHandle,
        0 // Core 0
    );

    if (result == pdPASS)
    {
        Serial.println("ThingsBoard Client: Task created successfully");
        return true;
    }
    else
    {
        Serial.println("ThingsBoard Client: Failed to create task");
        return false;
    }
}

void ThingsBoardClient::stopTask()
{
    if (taskHandle != NULL)
    {
        vTaskDelete(taskHandle);
        taskHandle = NULL;
        Serial.println("ThingsBoard Client: Task stopped");
    }
}

bool ThingsBoardClient::isConnected()
{
    return (tb != nullptr && tb->connected());
}

bool ThingsBoardClient::sendTelemetryData(const char *key, double value)
{
    if (tb != nullptr && tb->connected())
    {
        return tb->sendTelemetryData(key, value);
    }
    return false;
}

bool ThingsBoardClient::sendTelemetryData(const char *key, int value)
{
    if (tb != nullptr && tb->connected())
    {
        return tb->sendTelemetryData(key, value);
    }
    return false;
}

bool ThingsBoardClient::sendTelemetryData(const char *key, bool value)
{
    if (tb != nullptr && tb->connected())
    {
        return tb->sendTelemetryData(key, value);
    }
    return false;
}

void ThingsBoardClient::thingsBoardTask(void *pvParameters)
{
    ThingsBoardClient *client = static_cast<ThingsBoardClient *>(pvParameters);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    Serial.println("ThingsBoard Client: Communication task started");

    while (1)
    {
        // Connect to ThingsBoard if not connected
        if (!client->connectToThingsBoard())
        {
            Serial.println("ThingsBoard Client: Connection failed, retrying...");
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5000));
            continue;
        }

        // Subscribe to RPC if not already subscribed
        if (!client->subscribed)
        {
            if (!client->subscribeToRPC())
            {
                Serial.println("ThingsBoard Client: RPC subscription failed, retrying...");
                vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2000));
                continue;
            }
        }

        // Process ThingsBoard communication
        client->processThingsBoardLoop();

        // Send telemetry data
        client->sendTelemetryLoop();

        // Yield to other tasks
        taskYIELD();

        // Wait before next iteration
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}

bool ThingsBoardClient::connectToThingsBoard()
{
    if (isConnected())
    {
        return true;
    }

    Serial.println("ThingsBoard Client: Connecting...");

    Preferences pref;
    pref.begin("wifi-config", false);
    String token = pref.getString("deviceToken", TOKEN);
    pref.end();

    if (!tb->connect(THINGSBOARD_SERVER, token.c_str(), THINGSBOARD_PORT))
    {
        Serial.println("ThingsBoard Client: Connection failed");
        return false;
    }

    Serial.println("ThingsBoard Client: Connected successfully!");

    // Recreate RPC object to clear any previous subscriptions
    recreateRPCObject();
    subscribed = false; // Reset subscription status
    return true;
}

bool ThingsBoardClient::subscribeToRPC()
{
    if (subscribed || rpc == nullptr)
    {
        return subscribed;
    }

    Serial.println("ThingsBoard Client: Subscribing to RPC...");

    // Subscribe to each RPC callback individually using the library's API
    RPC_Callback setTargetTempCallback(RPC_SET_TARGET_TEMP, processSetTargetTemp);
    if (!rpc->RPC_Subscribe(setTargetTempCallback))
    {
        Serial.println("ThingsBoard Client: Failed to subscribe setTargetTemp RPC");
        return false;
    }

    RPC_Callback setPidParamsCallback(RPC_SET_PID_PARAMS, processSetPidParams);
    if (!rpc->RPC_Subscribe(setPidParamsCallback))
    {
        Serial.println("ThingsBoard Client: Failed to subscribe setPidParams RPC");
        return false;
    }

    Serial.println("ThingsBoard Client: RPC subscription successful");
    subscribed = true;
    return true;
}

void ThingsBoardClient::sendTelemetryLoop()
{
    unsigned long currentTime = millis();
    if (currentTime - lastTelemetrySend >= TB_SEND_INTERVAL)
    {
        if (dataMutex != NULL && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) // Increased timeout
        {
            if (temperatureSensor != nullptr && pidController != nullptr &&
                temperatureSensor->isConnectedInternal() && isConnected())
            {
                // Get current values using internal functions while holding mutex
                double currentTemp = temperatureSensor->getCurrentTemperatureInternal();
                double targetTemp = pidController->getTargetTemperatureInternal();
                int pwmValue = pidController->getPWMValueInternal();
                double error = pidController->getErrorInternal();
                double pidOutput = pidController->getPIDOutputInternal();
                bool sensorConnected = temperatureSensor->isConnectedInternal();

                double kp, ki, kd;
                pidController->getPIDParametersInternal(kp, ki, kd);

                xSemaphoreGive(dataMutex);

                // Send telemetry data using the library's API (outside mutex)
                tb->sendTelemetryData("temperature", currentTemp);
                tb->sendTelemetryData("targetTemp", targetTemp);
                tb->sendTelemetryData("pwmValue", pwmValue);
                tb->sendTelemetryData("pwmPercent", map(pwmValue, 0, 1023, 0, 100));
                tb->sendTelemetryData("error", error);
                tb->sendTelemetryData("pidOutput", pidOutput);
                tb->sendTelemetryData("sensorConnected", sensorConnected);
                tb->sendTelemetryData("kp", kp);
                tb->sendTelemetryData("ki", ki);
                tb->sendTelemetryData("kd", kd);
                tb->sendTelemetryData("currentIP", wifiManager->getCurrentIP());
                tb->sendAttributeData("targetTemp", targetTemp);

                lastTelemetrySend = currentTime;
            }
            else
            {
                xSemaphoreGive(dataMutex);
            }
        }
        else
        {
            Serial.println("ThingsBoard Client: Failed to acquire mutex for telemetry");
        }
    }
}

void ThingsBoardClient::processThingsBoardLoop()
{
    // Process ThingsBoard loop with timeout protection
    unsigned long loopStart = millis();
    tb->loop();

    // Prevent tb.loop() from blocking too long
    if (millis() - loopStart > 100)
    {
        Serial.println("ThingsBoard Client: Loop took too long, yielding...");
        taskYIELD();
    }
}

// Static RPC callback functions
void ThingsBoardClient::processSetTargetTemp(const JsonVariantConst &data, JsonDocument &response)
{
    if (instance != nullptr)
    {
        instance->handleSetTargetTemp(data, response);
    }
    else
    {
        response["status"] = "error";
        response["message"] = "Client instance not available";
    }
}

void ThingsBoardClient::processSetPidParams(const JsonVariantConst &data, JsonDocument &response)
{
    if (instance != nullptr)
    {
        instance->handleSetPidParams(data, response);
    }
    else
    {
        response["status"] = "error";
        response["message"] = "Client instance not available";
    }
}

// Instance methods for handling RPC
void ThingsBoardClient::handleSetTargetTemp(const JsonVariantConst &data, JsonDocument &response)
{
    Serial.println("ThingsBoard Client: Received setTargetTemp RPC request");

    // Extract the new target temperature
    const double newTargetTemp = double(data);

    saveTargetTemperature(newTargetTemp);

    // Validate temperature range
    if (newTargetTemp > 0 && newTargetTemp < 60)
    {
        if (pidController != nullptr)
        {
            pidController->setTargetTemperature(newTargetTemp);

            Serial.println("ThingsBoard Client: Target temperature changed to " + String(newTargetTemp) + "°C");

            // Send success response
            response["status"] = "success";
            response["message"] = "Target temperature updated";
            response["newTargetTemp"] = newTargetTemp;
            return;
        }
        else
        {
            response["status"] = "error";
            response["message"] = "PID controller not available";
            return;
        }
    }

    // Send error response
    response["status"] = "error";
    response["message"] = "Invalid temperature range (0-60°C)";
    response["receivedValue"] = newTargetTemp;
}

void ThingsBoardClient::handleSetPidParams(const JsonVariantConst &data, JsonDocument &response)
{
    Serial.println("ThingsBoard Client: Received setPidParams RPC request");
    String jsonString = data.as<String>();
    Serial.println("  Data: " + jsonString);

    // Extract PID parameters using helper function
    const float newKp = extractFloatFromJson(jsonString, "kp");
    const float newKi = extractFloatFromJson(jsonString, "ki");
    const float newKd = extractFloatFromJson(jsonString, "kd");

    savePIDParameters(newKp, newKi, newKd);

    // Debug: Print extracted values
    Serial.println("Debug: Extracted values:");
    Serial.println("  newKp: " + String(newKp));
    Serial.println("  newKi: " + String(newKi));
    Serial.println("  newKd: " + String(newKd));

    // Validate PID parameters (basic validation)
    if (newKp >= 0 && newKi >= 0 && newKd >= 0)
    {
        if (pidController != nullptr)
        {
            pidController->setPIDParameters(newKp, newKi, newKd);

            Serial.println("ThingsBoard Client: PID Parameters Updated:");
            Serial.println("  Kp: " + String(newKp));
            Serial.println("  Ki: " + String(newKi));
            Serial.println("  Kd: " + String(newKd));

            // Send success response
            response["status"] = "success";
            response["message"] = "PID parameters updated";
            response["newKp"] = newKp;
            response["newKi"] = newKi;
            response["newKd"] = newKd;
            return;
        }
        else
        {
            response["status"] = "error";
            response["message"] = "PID controller not available";
            return;
        }
    }

    // Send error response
    response["status"] = "error";
    response["message"] = "Invalid PID parameters (must be >= 0)";
    response["receivedKp"] = newKp;
    response["receivedKi"] = newKi;
    response["receivedKd"] = newKd;
}

float ThingsBoardClient::extractFloatFromJson(const String &jsonString, const String &key)
{
    // Create the search pattern: "key":
    String searchKey = "\"" + key + "\":";

    // Find the key in the JSON string
    int keyStart = jsonString.indexOf(searchKey);
    if (keyStart == -1)
    {
        return 0.0; // Key not found
    }

    // Move past the key and colon
    int valueStart = keyStart + searchKey.length();

    // Skip any whitespace after the colon
    while (valueStart < jsonString.length() && jsonString.charAt(valueStart) == ' ')
    {
        valueStart++;
    }

    // Find the end of the value (comma or closing brace)
    int valueEnd = jsonString.indexOf(',', valueStart);
    if (valueEnd == -1)
    {
        valueEnd = jsonString.indexOf('}', valueStart);
    }

    if (valueEnd == -1)
    {
        return 0.0; // Malformed JSON
    }

    // Extract the value substring and convert to float
    String valueStr = jsonString.substring(valueStart, valueEnd);
    valueStr.trim(); // Remove any leading/trailing whitespace

    return valueStr.toFloat();
}

void ThingsBoardClient::recreateRPCObject()
{
    Serial.println("ThingsBoard Client: Recreating RPC object to clear previous subscriptions...");

    // Delete the old RPC object if it exists
    if (rpc != nullptr)
    {
        delete rpc;
        rpc = nullptr;
    }

    // Create a new RPC object (this clears all previous subscriptions)
    rpc = new Server_Side_RPC<MAX_RPC_SUBSCRIPTIONS, MAX_RPC_RESPONSE>();

    // Subscribe the RPC API implementation after construction
    tb->Subscribe_API_Implementation(*rpc);

    if (rpc == nullptr)
    {
        Serial.println("ThingsBoard Client: ERROR - Failed to create new RPC object!");
        return;
    }

    Serial.println("ThingsBoard Client: RPC object recreated successfully");
}

void ThingsBoardClient::saveTargetTemperature(double targetTemp)
{
    preferences.putDouble("targetTemp", targetTemp);
    Serial.println("ThingsBoard Client: Target temperature saved to preferences: " + String(targetTemp) + "°C");
}
void ThingsBoardClient::savePIDParameters(double kp, double ki, double kd)
{
    preferences.putDouble("Kp", kp);
    preferences.putDouble("Ki", ki);
    preferences.putDouble("Kd", kd);
    Serial.println("ThingsBoard Client: PID parameters saved to preferences");
}
