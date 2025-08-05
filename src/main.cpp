/*
 * ESP32 Smart Cooling System with PID Temperature Control and ThingsBoard Integration
 *
 * This modular system maintains a Peltier cooling chip at exactly 25°C using:
 * - DS18B20 temperature sensor on pin D2 (GPIO2)
 * - PWM control on pin D15 (GPIO15) for MOSFET module
 * - PID algorithm for precise temperature control
 * - ThingsBoard IoT platform integration for remote monitoring and control
 * - FreeRTOS for real-time multitasking
 *
 * Features:
 * - Modular design with separate classes for each major component
 * - Thread-safe communication between modules
 * - WiFi connectivity with auto-reconnection
 * - Real-time PID control using FreeRTOS tasks
 * - Remote control via ThingsBoard RPC
 * - Comprehensive telemetry data streaming
 *
 * Architecture:
 * - config.h: All configuration constants
 * - wifi_manager: WiFi connection management
 * - temperature_sensor: DS18B20 sensor handling with FreeRTOS task
 * - pid_controller: PID control logic with PWM output
 * - thingsboard_client: IoT communication and RPC handling
 * - main.cpp: System coordination and initialization
 *
 * Hardware Requirements:
 * - ESP32 Development Board
 * - DS18B20 Temperature Sensor (on cooling chip)
 * - MOSFET PWM Module for 12V Peltier chip control
 * - 4.7kΩ pull-up resistor for DS18B20
 * - 12V Peltier cooling chip
 *
 * Libraries Required:
 * - ThingsBoard (install from Arduino Library Manager)
 * - OneWire
 * - DallasTemperature
 * - WiFi
 * - ArduinoJson
 */

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// Include our modular components
#include "config.h"
#include "wifi_manager.h"
#include "temperature_sensor.h"
#include "pid_controller.h"
#include "thingsboard_client.h"

// System components
WiFiManager wifiManager;
TemperatureSensor temperatureSensor(TEMP_SENSOR_PIN);
PIDController pidController;
ThingsBoardClient thingsBoardClient;

// Global mutex for thread-safe data sharing
SemaphoreHandle_t xDataMutex;

// Task handle for the coordination task
TaskHandle_t coordinationTaskHandle = NULL;

// Function prototypes
void initializeSystem();
bool startAllTasks();
void coordinationTask(void *pvParameters);

void setup()
{
  Serial.begin(115200);
  delay(1000); // Give serial monitor time to connect

  Serial.println("=== ESP32 Smart Cooling System Starting ===");
  Serial.println("Modular Architecture with ThingsBoard Integration");
  Serial.println();

  // Initialize the entire system
  initializeSystem();

  Serial.println("=== System Initialization Complete ===");
  Serial.println("All modules are running in their own tasks");
  Serial.println("Check ThingsBoard dashboard for real-time data");
  Serial.println();
}

void loop()
{
  // Main loop is empty - all work is done in FreeRTOS tasks
  // Just print a heartbeat every 30 seconds
  static unsigned long lastHeartbeat = 0;

  if (millis() - lastHeartbeat > 30000)
  {
    Serial.println("System heartbeat - all tasks running");
    lastHeartbeat = millis();
  }

  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void initializeSystem()
{
  Serial.println("Creating system mutex...");

  // Create mutex for thread-safe data sharing
  xDataMutex = xSemaphoreCreateMutex();
  if (xDataMutex == NULL)
  {
    Serial.println("FATAL ERROR: Failed to create system mutex!");
    while (1)
    {
      delay(1000);
    }
  }
  Serial.println("✓ System mutex created");

  // Initialize WiFi
  Serial.println("Initializing WiFi...");
  if (!wifiManager.begin())
  {
    Serial.println("WARNING: WiFi initialization failed!");
    Serial.println("System will continue and retry connection automatically");
  }
  else
  {
    Serial.println("✓ WiFi connected successfully");
    wifiManager.printConnectionInfo();
  }

  // Initialize Temperature Sensor
  Serial.println("Initializing temperature sensor...");
  if (!temperatureSensor.begin())
  {
    Serial.println("WARNING: Temperature sensor initialization failed!");
    Serial.println("System will continue but temperature control will be disabled");
  }
  else
  {
    Serial.println("✓ Temperature sensor initialized");
  }

  // Initialize PID Controller
  Serial.println("Initializing PID controller...");
  if (!pidController.begin())
  {
    Serial.println("ERROR: PID controller initialization failed!");
    while (1)
    {
      delay(1000);
    }
  }
  else
  {
    Serial.println("✓ PID controller initialized");
  }

  // Initialize ThingsBoard Client
  Serial.println("Initializing ThingsBoard client...");
  if (!thingsBoardClient.begin())
  {
    Serial.println("WARNING: ThingsBoard client initialization failed!");
    Serial.println("System will continue without IoT connectivity");
  }
  else
  {
    Serial.println("✓ ThingsBoard client initialized");
  }

  // Start all FreeRTOS tasks
  if (!startAllTasks())
  {
    Serial.println("FATAL ERROR: Failed to start system tasks!");
    while (1)
    {
      delay(1000);
    }
  }
}

bool startAllTasks()
{
  Serial.println("Starting FreeRTOS tasks...");

  // Start temperature sensor task
  if (!temperatureSensor.startTask(xDataMutex))
  {
    Serial.println("ERROR: Failed to start temperature sensor task");
    return false;
  }
  Serial.println("✓ Temperature sensor task started");

  // Start PID controller task
  if (!pidController.startTask(xDataMutex))
  {
    Serial.println("ERROR: Failed to start PID controller task");
    return false;
  }
  Serial.println("✓ PID controller task started");

  // Start ThingsBoard client task
  if (!thingsBoardClient.startTask(xDataMutex, &pidController, &temperatureSensor))
  {
    Serial.println("ERROR: Failed to start ThingsBoard client task");
    return false;
  }
  Serial.println("✓ ThingsBoard client task started");

  // Start coordination task
  BaseType_t result = xTaskCreatePinnedToCore(
      coordinationTask,
      "CoordinationTask",
      4096,
      NULL,
      2, // Medium priority
      &coordinationTaskHandle,
      1 // Core 1
  );

  if (result != pdPASS)
  {
    Serial.println("ERROR: Failed to start coordination task");
    return false;
  }
  Serial.println("✓ Coordination task started");

  Serial.println("All tasks started successfully!");
  return true;
}

void coordinationTask(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  unsigned long lastHeartbeat = 0;

  Serial.println("Coordination task: Started");

  while (1)
  {
    // Print periodic heartbeat from coordination task
    if (millis() - lastHeartbeat > 30000)
    {
      Serial.println("Coordination task: Running normally");
      lastHeartbeat = millis();
    }

    // Ensure WiFi connection
    if (!wifiManager.ensureConnection())
    {
      Serial.println("Coordination task: WiFi connection lost, retrying...");
    }

    // Get temperature data and update PID controller
    if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(200)) == pdTRUE) // Increased timeout
    {
      // Get temperature data using internal functions while holding mutex
      double currentTemp = temperatureSensor.getCurrentTemperatureInternal();
      bool sensorConnected = temperatureSensor.isConnectedInternal();

      // Update PID controller using internal function while holding mutex
      pidController.updateTemperatureInternal(currentTemp, sensorConnected);

      xSemaphoreGive(xDataMutex);

      // Optional: Output debug data to serial (uncomment if needed)
      // pidController.outputSerialData(currentTemp);
    }
    else
    {
      Serial.println("Coordination task: Failed to acquire mutex - system may be overloaded");
    }

    // Yield to other tasks
    taskYIELD();

    // Run coordination every 500ms
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
  }
}