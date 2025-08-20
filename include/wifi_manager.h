#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>

class WiFiManager
{
public:
    // Constructor
    WiFiManager();

    // Destructor
    ~WiFiManager();

    // Initialize WiFi connection or start config portal
    bool begin();

    // Check if connected and reconnect if needed
    bool ensureConnection();

    // Get connection status
    bool isConnected();

    // Get IP address
    IPAddress getLocalIP();

    // Get current IP address as string
    const char *getCurrentIP();

    // Get current IP address
    char current_ip[20];

    // Print connection info
    void printConnectionInfo();

    // Start configuration portal
    void startConfigPortal();

    // Stop configuration portal and try to connect
    bool stopConfigPortal();

    // Check if in config mode
    bool isConfigMode();

    // Start permanent web server for WiFi management
    void startPermanentWebServer();

    // Stop permanent web server
    void stopPermanentWebServer();

    // Handle web server in main loop
    void handleWebServer();

private:
    // Preferences for storing WiFi credentials
    Preferences preferences;

    // Web server for configuration
    AsyncWebServer *server;

    // Permanent web server for ongoing management
    AsyncWebServer *permanentServer;

    // Configuration mode flag
    bool configMode;

    // Current WiFi credentials
    String ssid;
    String password;

    // Internal connection attempt
    bool connectToWiFi();

    // Load WiFi credentials from preferences
    bool loadCredentials();

    // Save WiFi credentials to preferences
    void saveCredentials(const String &ssid, const String &password);

    // Start access point
    void startAccessPoint();

    // Setup web server routes
    void setupWebServer();

    // Generate configuration webpage HTML
    String generateConfigPage();

    // Generate success page HTML
    String generateSuccessPage();

    // Maximum connection attempts
    static const int MAX_CONNECTION_ATTEMPTS = 40;

    // Connection timeout per attempt (ms)
    static const int CONNECTION_TIMEOUT = 500;

    // Access point credentials
    static const char *AP_SSID;
    static const char *AP_PASSWORD;
};

#endif // WIFI_MANAGER_H