#ifndef TEMPERATURE_SENSOR_H
#define TEMPERATURE_SENSOR_H

#include <OneWire.h>
#include <DallasTemperature.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

class TemperatureSensor
{
public:
    // Constructor
    TemperatureSensor(int pin);

    // Destructor
    ~TemperatureSensor();

    // Initialize the sensor
    bool begin();

    // Start the temperature reading task
    bool startTask(SemaphoreHandle_t dataMutex);

    // Stop the temperature reading task
    void stopTask();

    // Get current temperature (thread-safe)
    double getCurrentTemperature();

    // Check if sensor is connected
    bool isConnected();

    // Get last reading timestamp
    unsigned long getLastReadingTime();

    // Get device count
    int getDeviceCount();

    // Internal getters (non-mutex) for use when mutex is already held
    double getCurrentTemperatureInternal() const { return currentTemp; }
    bool isConnectedInternal() const { return sensorConnected; }
    unsigned long getLastReadingTimeInternal() const { return lastReadingTime; }

private:
    // OneWire and DallasTemperature objects
    OneWire *oneWire;
    DallasTemperature *tempSensor;

    // Pin number
    int sensorPin;

    // Task handle
    TaskHandle_t taskHandle;

    // Shared data (protected by mutex)
    double currentTemp;
    bool sensorConnected;
    unsigned long lastReadingTime;

    // Mutex for data protection
    SemaphoreHandle_t dataMutex;

    // Task function (static)
    static void temperatureReadingTask(void *pvParameters);

    // Internal reading function
    void readTemperature();

    // Validate temperature reading
    bool isValidTemperature(float temp);
};

#endif // TEMPERATURE_SENSOR_H