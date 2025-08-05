#ifndef CONFIG_H
#define CONFIG_H

// WiFi Configuration Portal Settings
constexpr char AP_SSID[] = "ESP32-SmartCooling";
constexpr char AP_PASSWORD[] = "12345678"; // Minimum 8 characters for WPA2
constexpr int CONFIG_PORTAL_TIMEOUT = 300; // 5 minutes timeout for config portal

// ThingsBoard Configuration
constexpr char TOKEN[] = "gn50bkmog39razmpvoc2";
constexpr char THINGSBOARD_SERVER[] = "demo.thingsboard.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;
constexpr uint16_t MAX_MESSAGE_SEND_SIZE = 256U;
constexpr uint16_t MAX_MESSAGE_RECEIVE_SIZE = 256U;
constexpr uint8_t MAX_RPC_SUBSCRIPTIONS = 2U;
constexpr uint8_t MAX_RPC_RESPONSE = 5U;

// Pin Definitions
#define TEMP_SENSOR_PIN 14 // D14 - Temperature sensor data pin
#define PWM_OUTPUT_PIN 16 // D16 - PWM output to MOSFET module
#define PWM_CHANNEL 0     // LEDC channel for PWM output

// Temperature Control Settings
#define DEFAULT_TARGET_TEMP 25.0 // Default target temperature in Celsius
#define PWM_FREQUENCY 10         // PWM frequency in Hz (increased from 10Hz)
#define PWM_RESOLUTION 10        // 10-bit resolution (0-1023)

// Safety and limits
#define MIN_PWM 0
#define MAX_PWM 1023
#define MAX_TEMP_DIFF 50.0
#define MAX_SAFE_TEMP 60.0   // Maximum safe operating temperature
#define MIN_SAFE_TEMP -10.0  // Minimum safe operating temperature
#define MUTEX_TIMEOUT_MS 200 // Standard mutex timeout

// Timing intervals (in milliseconds)
#define TEMP_READING_INTERVAL 500 // Temperature reading interval
#define TB_SEND_INTERVAL 2000     // ThingsBoard telemetry send interval
#define PID_SAMPLE_TIME 500       // PID calculation interval

// RPC method names
constexpr const char RPC_SET_TARGET_TEMP[] = "setTargetTemp";
constexpr const char RPC_SET_PID_PARAMS[] = "setPidParams";
constexpr const char RPC_TARGET_TEMP_KEY[] = "value";
constexpr const char RPC_KP_KEY[] = "kp";
constexpr const char RPC_KI_KEY[] = "ki";
constexpr const char RPC_KD_KEY[] = "kd";

#endif // CONFIG_H