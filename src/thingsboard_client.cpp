#include "thingsboard_client.h"
#include "config.h"
#include "pid_controller.h"
#include "temperature_sensor.h"
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
}

bool ThingsBoardClient::begin()
{
    Serial.println("ThingsBoard Client: Initializing...");

    if (tb == nullptr || rpc == nullptr || mqttClient == nullptr)
    {
        Serial.println("ThingsBoard Client: Failed to create ThingsBoard objects");
        return false;
    }

    Serial.println("ThingsBoard Client: Initialization complete");
    return true;
}

bool ThingsBoardClient::startTask(SemaphoreHandle_t mutex, PIDController *pid, TemperatureSensor *temp)
{
    if (taskHandle != NULL)
    {
        Serial.println("ThingsBoard Client: Task already running");
        return false;
    }

    dataMutex = mutex;
    pidController = pid;
    temperatureSensor = temp;

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
    if (!tb->connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT))
    {
        Serial.println("ThingsBoard Client: Connection failed");
        return false;
    }

    Serial.println("ThingsBoard Client: Connected successfully!");
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
    const float newTargetTemp = data[RPC_TARGET_TEMP_KEY];

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

    // Extract PID parameters
    const float newKp = data[RPC_KP_KEY];
    const float newKi = data[RPC_KI_KEY];
    const float newKd = data[RPC_KD_KEY];

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