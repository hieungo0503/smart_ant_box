#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <Preferences.h>

class PIDController
{
public:
    // Constructor
    PIDController();

    // Destructor
    ~PIDController();

    // Initialize the PID controller and PWM
    bool begin();

    // Start the PID control task
    bool startTask(SemaphoreHandle_t dataMutex);

    // Stop the PID control task
    void stopTask();

    // Set PID parameters (thread-safe)
    void setPIDParameters(double kp, double ki, double kd);

    // Get PID parameters (thread-safe)
    void getPIDParameters(double &kp, double &ki, double &kd);

    // Set target temperature (thread-safe)
    void setTargetTemperature(double target);

    // Get target temperature (thread-safe)
    double getTargetTemperature();

    // Update current temperature and perform PID calculation
    void updateTemperature(double currentTemp, bool sensorConnected);

    // Internal update function (non-mutex) for use when mutex is already held
    void updateTemperatureInternal(double currentTemp, bool sensorConnected);

    // Get current PID output values
    double getError();
    double getPIDOutput();
    int getPWMValue();

    // Output data for serial plotter
    void outputSerialData(double currentTemp);

    // Internal getters (non-mutex) for use when mutex is already held
    double getTargetTemperatureInternal() const { return targetTemp; }
    void getPIDParametersInternal(double &kp, double &ki, double &kd) const
    {
        kp = Kp;
        ki = Ki;
        kd = Kd;
    }
    double getErrorInternal() const { return error; }
    double getPIDOutputInternal() const { return pidOutput; }
    int getPWMValueInternal() const { return pwmValue; }

private:
    // PID parameters
    double Kp, Ki, Kd;
    double targetTemp;

    // PID calculation variables
    double error;
    double lastError;
    double integral;
    double derivative;
    double pidOutput;
    int pwmValue;

    // Timing
    unsigned long lastPIDTime;

    // Task handle
    TaskHandle_t taskHandle;

    // Mutex for data protection
    SemaphoreHandle_t dataMutex;

    // preferences for storing PID parameters and target temperature
    Preferences preferences;

    // Task function (static)
    static void pidControlTask(void *pvParameters);

    // Internal PID calculation
    void calculatePID();

    // Apply PWM output
    void applyPWM();

    // Initialize PWM channel
    bool initializePWM();

    // Safety check for temperature difference
    bool isTemperatureSafe(double currentTemp);

    // Temperature validation helper
    bool isValidTemperature(double temp);

    // Load PID parameters from preferences
    bool loadParameters();
};

#endif // PID_CONTROLLER_H