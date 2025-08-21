#include "pid_controller.h"
#include "config.h"
#include <Arduino.h>
#include <esp32-hal-ledc.h>

PIDController::PIDController()
    : error(0.0), lastError(0.0), integral(0.0), derivative(0.0),
      pidOutput(0.0), pwmValue(0), lastPIDTime(0),
      taskHandle(NULL), dataMutex(NULL)
{
}

PIDController::~PIDController()
{
    stopTask();
    preferences.end();
}

bool PIDController::begin()
{
    // Initialize Preferences for storing PID parameters and target temperature
    preferences.begin("smart_config", false);
    // Load saved PID parameters and target temperature
    loadParameters();

    Serial.println("PID Controller: Initializing...");

    if (!initializePWM())
    {
        Serial.println("PID Controller: Failed to initialize PWM");
        return false;
    }

    lastPIDTime = millis();
    Serial.println("PID Controller: Initialization complete");
    Serial.print("PID Controller: Target temperature: ");
    Serial.print(targetTemp);
    Serial.println("°C");

    return true;
}

bool PIDController::startTask(SemaphoreHandle_t mutex)
{
    if (taskHandle != NULL)
    {
        Serial.println("PID Controller: Task already running");
        return false;
    }

    dataMutex = mutex;

    BaseType_t result = xTaskCreatePinnedToCore(
        pidControlTask,
        "PIDControlTask",
        4096,
        this, // Pass this instance as parameter
        2,    // Medium priority for PID control
        &taskHandle,
        1 // Core 1
    );

    if (result == pdPASS)
    {
        Serial.println("PID Controller: Task created successfully");
        return true;
    }
    else
    {
        Serial.println("PID Controller: Failed to create task");
        return false;
    }
}

void PIDController::stopTask()
{
    if (taskHandle != NULL)
    {
        // Turn off PWM before stopping
        ledcWrite(PWM_CHANNEL, 0);
        vTaskDelete(taskHandle);
        taskHandle = NULL;
        Serial.println("PID Controller: Task stopped");
    }
}

void PIDController::setPIDParameters(double kp, double ki, double kd)
{
    // Input validation
    if (kp < 0 || ki < 0 || kd < 0)
    {
        Serial.println("PID Controller: Error - PID parameters must be non-negative");
        return;
    }

    if (kp > 1000 || ki > 100 || kd > 100)
    {
        Serial.println("PID Controller: Warning - PID parameters seem unusually high");
    }

    if (dataMutex != NULL && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) // Increased from 100ms
    {
        Kp = kp;
        Ki = ki;
        Kd = kd;

        // Reset integral to prevent windup
        integral = 0;

        Serial.println("PID Controller: Parameters updated");
        Serial.println("  Kp: " + String(Kp));
        Serial.println("  Ki: " + String(Ki));
        Serial.println("  Kd: " + String(Kd));

        xSemaphoreGive(dataMutex);
    }
    else
    {
        Serial.println("PID Controller: Failed to acquire mutex for parameter update");
    }
}

void PIDController::getPIDParameters(double &kp, double &ki, double &kd)
{
    if (dataMutex != NULL && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) // Increased from 50ms
    {
        kp = Kp;
        ki = Ki;
        kd = Kd;
        xSemaphoreGive(dataMutex);
    }
    else
    {
        kp = Kp;
        ki = Ki;
        kd = Kd;
    }
}

void PIDController::setTargetTemperature(double target)
{
    // Input validation for temperature range
    if (target < 0 || target > 60)
    {
        Serial.println("PID Controller: Error - Target temperature out of safe range (0-60°C)");
        return;
    }

    if (dataMutex != NULL && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) // Increased from 100ms
    {
        targetTemp = target;

        // Reset integral to prevent windup when changing target
        integral = 0;

        Serial.println("PID Controller: Target temperature changed to " + String(targetTemp) + "°C");

        xSemaphoreGive(dataMutex);
    }
    else
    {
        Serial.println("PID Controller: Failed to acquire mutex for target temperature update");
    }
}

double PIDController::getTargetTemperature()
{
    if (dataMutex == NULL)
        return targetTemp;

    double target = 0.0;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) // Increased from 50ms
    {
        target = targetTemp;
        xSemaphoreGive(dataMutex);
    }
    return target;
}

double PIDController::getError()
{
    return error;
}

double PIDController::getPIDOutput()
{
    return pidOutput;
}

int PIDController::getPWMValue()
{
    return pwmValue;
}

void PIDController::outputSerialData(double currentTemp)
{
    // Output data for Serial Plotter
    Serial.print("CurrentTemp:");
    Serial.print(currentTemp, 2);
    Serial.print(",TargetTemp:");
    Serial.print(targetTemp, 2);
    Serial.print(",PWMPercent:");
    Serial.print(map(pwmValue, 0, 1023, 0, 100));
    Serial.print(",Error:");
    Serial.println(error, 2);
}

void PIDController::pidControlTask(void *pvParameters)
{
    PIDController *controller = static_cast<PIDController *>(pvParameters);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    Serial.println("PID Controller: Control task started");

    while (1)
    {
        // The actual PID calculation will be triggered by updateTemperature()
        // This task just ensures regular execution timing

        // Yield to other tasks
        taskYIELD();

        // Wait for next PID calculation
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PID_SAMPLE_TIME));
    }
}

void PIDController::updateTemperature(double currentTemp, bool sensorConnected)
{
    if (dataMutex != NULL && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) // Increased from 50ms
    {
        updateTemperatureInternal(currentTemp, sensorConnected);
        xSemaphoreGive(dataMutex);
    }
    else
    {
        Serial.println("PID Controller: Failed to acquire mutex for temperature update");
    }
}

void PIDController::updateTemperatureInternal(double currentTemp, bool sensorConnected)
{
    if (sensorConnected && isValidTemperature(currentTemp))
    {
        // Calculate error (positive error means too hot, need more cooling)
        error = currentTemp - targetTemp;
        calculatePID();
        applyPWM();
    }
    else
    {
        // Safety: turn off cooling if sensor disconnected or invalid reading
        pwmValue = 0;
        integral = 0; // Reset integral when sensor fails
        ledcWrite(PWM_CHANNEL, 0);
        if (!sensorConnected)
        {
            Serial.println("PID Controller: Sensor disconnected - cooling disabled");
        }
        else
        {
            Serial.println("PID Controller: Invalid temperature reading - cooling disabled");
        }
    }
}

// Add helper function for temperature validation
bool PIDController::isValidTemperature(double temp)
{
    return (temp > -50 && temp < 100); // Reasonable temperature range for sensor
}

void PIDController::calculatePID()
{
    unsigned long currentTime = millis();
    double deltaTime = (currentTime - lastPIDTime) / 1000.0;

    if (deltaTime <= 0)
    {
        return; // Prevent division by zero
    }

    // Error is already calculated in updateTemperature()
    // (positive error means too hot, need more cooling)

    // Safety check - if temperature difference is too large, use maximum cooling
    if (abs(error) > MAX_TEMP_DIFF)
    {
        Serial.println("PID Controller: Warning - Large temperature difference detected!");
        if (error > 0)
        {
            pwmValue = MAX_PWM; // Maximum cooling
        }
        else
        {
            pwmValue = MIN_PWM; // No cooling needed
        }
        // Reset integral to prevent windup
        integral = 0;
        lastPIDTime = currentTime;
        return;
    }

    // Calculate integral (with windup protection)
    integral += error * deltaTime;

    // Integral windup protection
    double maxIntegral = 100.0;
    if (integral > maxIntegral)
        integral = maxIntegral;
    if (integral < -maxIntegral)
        integral = -maxIntegral;

    // Calculate derivative
    derivative = (error - lastError) / deltaTime;

    // Calculate PID output
    pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // Convert PID output to PWM value (0-1023)
    if (pidOutput > 0)
    {
        pwmValue = constrain(pidOutput, MIN_PWM, MAX_PWM);
    }
    else
    {
        pwmValue = MIN_PWM;
    }

    // Store values for next iteration
    lastError = error;
    lastPIDTime = currentTime;
}

void PIDController::applyPWM()
{
    ledcWrite(PWM_CHANNEL, pwmValue);
}

bool PIDController::initializePWM()
{
    ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(PWM_OUTPUT_PIN, PWM_CHANNEL);
    ledcWrite(PWM_CHANNEL, 0);

    Serial.println("PID Controller: PWM initialized");
    Serial.print("  Pin: ");
    Serial.println(PWM_OUTPUT_PIN);
    Serial.print("  Channel: ");
    Serial.println(PWM_CHANNEL);
    Serial.print("  Frequency: ");
    Serial.print(PWM_FREQUENCY);
    Serial.println(" Hz");
    Serial.print("  Resolution: ");
    Serial.print(PWM_RESOLUTION);
    Serial.println(" bits");

    return true;
}

bool PIDController::isTemperatureSafe(double currentTemp)
{
    return (abs(currentTemp - targetTemp) <= MAX_TEMP_DIFF);
}

bool PIDController::loadParameters()
{

    Kp = preferences.getDouble("Kp", DEFAULT_KP);
    Ki = preferences.getDouble("Ki", DEFAULT_KI);
    Kd = preferences.getDouble("Kd", DEFAULT_KD);
    targetTemp = preferences.getDouble("targetTemp", DEFAULT_TARGET_TEMP);

    Serial.println("PID Controller: Loaded parameters from preferences");
    Serial.print("  Kp: ");
    Serial.println(Kp);
    Serial.print("  Ki: ");
    Serial.println(Ki);
    Serial.print("  Kd: ");
    Serial.println(Kd);
    Serial.print("  Target Temp: ");
    Serial.println(targetTemp);

    return true;
}