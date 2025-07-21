#include "temperature_sensor.h"
#include "config.h"
#include <Arduino.h>

TemperatureSensor::TemperatureSensor(int pin) : sensorPin(pin), taskHandle(NULL),
                                                currentTemp(0.0), sensorConnected(false), lastReadingTime(0), dataMutex(NULL)
{
    oneWire = new OneWire(sensorPin);
    tempSensor = new DallasTemperature(oneWire);
}

TemperatureSensor::~TemperatureSensor()
{
    stopTask();
    delete tempSensor;
    delete oneWire;
}

bool TemperatureSensor::begin()
{
    Serial.println("Temperature Sensor: Initializing...");

    tempSensor->begin();
    int deviceCount = tempSensor->getDeviceCount();

    Serial.print("Temperature Sensor: Devices found: ");
    Serial.println(deviceCount);

    if (deviceCount == 0)
    {
        Serial.println("Temperature Sensor: No devices found!");
        return false;
    }

    tempSensor->setResolution(12);
    Serial.println("Temperature Sensor: Initialization complete");
    return true;
}

bool TemperatureSensor::startTask(SemaphoreHandle_t mutex)
{
    if (taskHandle != NULL)
    {
        Serial.println("Temperature Sensor: Task already running");
        return false;
    }

    dataMutex = mutex;

    BaseType_t result = xTaskCreatePinnedToCore(
        temperatureReadingTask,
        "TempSensorTask",
        4096,
        this, // Pass this instance as parameter
        3,    // High priority for temperature reading
        &taskHandle,
        0 // Core 0
    );

    if (result == pdPASS)
    {
        Serial.println("Temperature Sensor: Task created successfully");
        return true;
    }
    else
    {
        Serial.println("Temperature Sensor: Failed to create task");
        return false;
    }
}

void TemperatureSensor::stopTask()
{
    if (taskHandle != NULL)
    {
        vTaskDelete(taskHandle);
        taskHandle = NULL;
        Serial.println("Temperature Sensor: Task stopped");
    }
}

double TemperatureSensor::getCurrentTemperature()
{
    if (dataMutex == NULL)
        return currentTemp;

    double temp = 0.0;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) // Increased from 50ms
    {
        temp = currentTemp;
        xSemaphoreGive(dataMutex);
    }
    return temp;
}

bool TemperatureSensor::isConnected()
{
    if (dataMutex == NULL)
        return sensorConnected;

    bool connected = false;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) // Increased from 50ms
    {
        connected = sensorConnected;
        xSemaphoreGive(dataMutex);
    }
    return connected;
}

unsigned long TemperatureSensor::getLastReadingTime()
{
    if (dataMutex == NULL)
        return lastReadingTime;

    unsigned long time = 0;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) // Increased from 50ms
    {
        time = lastReadingTime;
        xSemaphoreGive(dataMutex);
    }
    return time;
}

int TemperatureSensor::getDeviceCount()
{
    return tempSensor->getDeviceCount();
}

void TemperatureSensor::temperatureReadingTask(void *pvParameters)
{
    TemperatureSensor *sensor = static_cast<TemperatureSensor *>(pvParameters);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    Serial.println("Temperature Sensor: Reading task started");

    while (1)
    {
        sensor->readTemperature();

        // Yield to other tasks
        taskYIELD();

        // Wait for next reading
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TEMP_READING_INTERVAL));
    }
}

void TemperatureSensor::readTemperature()
{
    // Request temperature reading
    tempSensor->requestTemperatures();

    // Small delay for temperature conversion (increased for 12-bit resolution)
    vTaskDelay(pdMS_TO_TICKS(20));

    float temp = tempSensor->getTempCByIndex(0);

    // Update global variables with mutex protection
    if (dataMutex != NULL && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) // Increased from 50ms
    {
        if (isValidTemperature(temp))
        {
            currentTemp = temp;
            sensorConnected = true;
            lastReadingTime = millis();
        }
        else
        {
            sensorConnected = false;
            Serial.print("Temperature Sensor: Invalid reading: ");
            Serial.println(temp);
        }
        xSemaphoreGive(dataMutex);
    }
    else
    {
        Serial.println("Temperature Sensor: Failed to acquire mutex for reading");
    }
}

bool TemperatureSensor::isValidTemperature(float temp)
{
    return (temp != DEVICE_DISCONNECTED_C && temp > -50 && temp < 100);
}