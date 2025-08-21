#include "wifi_manager.h"
#include "config.h"
#include <Arduino.h>
#include <DNSServer.h>

// Static member definitions
const char *WiFiManager::AP_SSID = "ESP32-SmartCooling";
const char *WiFiManager::AP_PASSWORD = "12345678";

WiFiManager::WiFiManager() : server(nullptr), permanentServer(nullptr), configMode(false)
{
    // Constructor - initialize server pointers
}

// Add destructor for proper cleanup
WiFiManager::~WiFiManager()
{
    if (server != nullptr)
    {
        server->end();
        delete server;
        server = nullptr;
    }
    if (permanentServer != nullptr)
    {
        permanentServer->end();
        delete permanentServer;
        permanentServer = nullptr;
    }
    preferences.end();
}

bool WiFiManager::begin()
{
    Serial.println("WiFi Manager: Initializing...");

    // Initialize preferences
    preferences.begin("wifi-config", false);

    // Try to load saved credentials
    if (loadCredentials())
    {
        Serial.println("WiFi Manager: Found saved credentials, attempting connection...");
        if (connectToWiFi())
        {
            return true;
        }
        else
        {
            Serial.println("WiFi Manager: Saved credentials failed, starting config portal...");
        }
    }
    else
    {
        Serial.println("WiFi Manager: No saved credentials found, starting config portal...");
    }

    // Start configuration portal
    startConfigPortal();
    return false; // Will return false until user configures WiFi
}

bool WiFiManager::ensureConnection()
{
    if (configMode)
    {
        // In config mode, don't try to reconnect
        return false;
    }

    if (isConnected())
    {
        return true;
    }

    Serial.println("WiFi Manager: Connection lost, reconnecting...");
    return connectToWiFi();
}

bool WiFiManager::isConnected()
{
    return (WiFi.status() == WL_CONNECTED && !configMode);
}

IPAddress WiFiManager::getLocalIP()
{
    if (configMode)
    {
        return WiFi.softAPIP();
    }
    return WiFi.localIP();
}

void WiFiManager::printConnectionInfo()
{
    if (configMode)
    {
        Serial.println("WiFi Manager: Running in configuration mode");
        Serial.print("WiFi Manager: AP SSID: ");
        Serial.println(AP_SSID);
        Serial.print("WiFi Manager: AP IP address: ");
        Serial.println(WiFi.softAPIP());
        Serial.println("WiFi Manager: Connect to configure WiFi credentials");
    }
    else if (isConnected())
    {
        Serial.println("WiFi Manager: Connected successfully");
        Serial.print("Device Token: ");
        Serial.println(deviceToken);
        Serial.print("WiFi Manager: SSID: ");
        Serial.println(ssid);
        Serial.print("WiFi Manager: IP address: ");
        Serial.println(getLocalIP());
        strcpy(current_ip, getLocalIP().toString().c_str());
        Serial.print("WiFi Manager: Signal strength: ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
    }
    else
    {
        Serial.println("WiFi Manager: Not connected");
    }
}

void WiFiManager::startConfigPortal()
{
    Serial.println("WiFi Manager: Starting configuration portal...");

    configMode = true;

    // Start access point
    startAccessPoint();

    // Create and start web server
    if (server == nullptr)
    {
        server = new AsyncWebServer(80);
        setupWebServer();
    }

    server->begin();

    Serial.println("WiFi Manager: Configuration portal started");
    printConnectionInfo();
}

bool WiFiManager::stopConfigPortal()
{
    Serial.println("WiFi Manager: Stopping configuration portal...");

    if (server != nullptr)
    {
        server->end();
        delete server;
        server = nullptr;
    }

    WiFi.softAPdisconnect(true);
    configMode = false;

    // Try to connect with configured credentials
    if (ssid.length() > 0 && password.length() > 0)
    {
        return connectToWiFi();
    }
    else
    {
        Serial.println("WiFi Manager: No valid credentials to connect");
        return false;
    }
}

bool WiFiManager::isConfigMode()
{
    return configMode;
}

bool WiFiManager::connectToWiFi()
{
    if (ssid.length() == 0)
    {
        Serial.println("WiFi Manager: No SSID configured");
        return false;
    }

    Serial.println("WiFi Manager: Connecting to WiFi...");
    Serial.print("WiFi Manager: SSID: ");
    Serial.println(ssid);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), password.c_str());

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < MAX_CONNECTION_ATTEMPTS)
    {
        delay(CONNECTION_TIMEOUT);
        Serial.print(".");
        attempts++;
    }

    Serial.println(); // New line after dots

    if (WiFi.status() == WL_CONNECTED)
    {
        configMode = false;
        printConnectionInfo();
        startPermanentWebServer();
        return true;
    }
    else
    {
        Serial.println("WiFi Manager: Connection failed!");
        Serial.print("WiFi Manager: Final status: ");
        Serial.println(WiFi.status());
        return false;
    }
}

bool WiFiManager::loadCredentials()
{
    ssid = preferences.getString("ssid", "");
    password = preferences.getString("password", "");
    deviceToken = preferences.getString("deviceToken", TOKEN);

    return (ssid.length() > 0);
}

void WiFiManager::saveCredentials(const String &newSsid, const String &newPassword, const String &newDeviceToken)
{
    // Save credentials to preferences
    preferences.putString("ssid", newSsid);
    preferences.putString("password", newPassword);
    preferences.putString("deviceToken", newDeviceToken);

    ssid = newSsid;
    password = newPassword;
    deviceToken = newDeviceToken;

    Serial.println("WiFi Manager: Credentials saved");
}

void WiFiManager::startAccessPoint()
{
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);

    delay(2000); // Give AP time to start

    Serial.print("WiFi Manager: Access Point started - SSID: ");
    Serial.println(AP_SSID);
    Serial.print("WiFi Manager: IP address: ");
    Serial.println(WiFi.softAPIP());
}

void WiFiManager::setupWebServer()
{
    // Serve the main configuration page
    server->on("/", HTTP_GET, [this](AsyncWebServerRequest *request)
               { request->send(200, "text/html", generateConfigPage()); });

    // Handle WiFi configuration submission
    server->on("/configure", HTTP_POST, [this](AsyncWebServerRequest *request)
               {
        String newSsid = "";
        String newPassword = "";
        String newDeviceToken = "";
        
        if (request->hasParam("ssid", true))
        {
            newSsid = request->getParam("ssid", true)->value();
        }
        
        if (request->hasParam("password", true))
        {
            newPassword = request->getParam("password", true)->value();
        }

        if (request->hasParam("deviceToken", true))
        {
            newDeviceToken = request->getParam("deviceToken", true)->value();
        }
        
        if (newSsid.length() > 0 && newDeviceToken.length() > 0)
        {
            saveCredentials(newSsid, newPassword, newDeviceToken);
            request->send(200, "text/html", generateSuccessPage());
            
            // Delay and restart to apply new settings
            delay(3000);
            ESP.restart();
        }
        else
        {
            request->send(400, "text/html", "<h1>Error: SSID cannot be empty!</h1><a href='/'>Go back</a>");
        } });

    // Handle reset credentials
    server->on("/reset", HTTP_GET, [this](AsyncWebServerRequest *request)
               {
        preferences.clear();
        ssid = "";
        password = "";
        deviceToken = "";
        request->send(200, "text/html", "<h1>Credentials Reset!</h1><p>Device will restart...</p>");
        delay(3000);
        ESP.restart(); });

    // Serve scan results
    server->on("/scan", HTTP_GET, [this](AsyncWebServerRequest *request)
               {
        String json = "[";
        int n = WiFi.scanNetworks();
        for (int i = 0; i < n; ++i)
        {
            if (i) json += ",";
            json += "{";
            json += "\"ssid\":\"" + WiFi.SSID(i) + "\",";
            json += "\"rssi\":" + String(WiFi.RSSI(i)) + ",";
            json += "\"secure\":" + String(WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
            json += "}";
        }
        json += "]";
        request->send(200, "application/json", json); });
}

String WiFiManager::generateConfigPage()
{
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32 Smart Cooling System - WiFi Setup</title>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial, sans-serif; margin: 0; padding: 20px; background: #f0f2f5; }
        .container { max-width: 500px; margin: 0 auto; background: white; padding: 30px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
        h1 { color: #333; text-align: center; margin-bottom: 30px; }
        .form-group { margin-bottom: 20px; }
        label { display: block; margin-bottom: 5px; font-weight: bold; color: #555; }
        input[type="text"], input[type="password"], select { width: 100%; padding: 12px; border: 2px solid #ddd; border-radius: 5px; font-size: 16px; box-sizing: border-box; }
        input:focus, select:focus { border-color: #4CAF50; outline: none; }
        button { background: #4CAF50; color: white; padding: 12px 25px; border: none; border-radius: 5px; font-size: 16px; cursor: pointer; width: 100%; margin-top: 10px; }
        button:hover { background: #45a049; }
        .scan-btn { background: #2196F3; margin-bottom: 10px; }
        .scan-btn:hover { background: #1976D2; }
        .reset-btn { background: #f44336; }
        .reset-btn:hover { background: #d32f2f; }
        .network-list { max-height: 200px; overflow-y: auto; border: 1px solid #ddd; border-radius: 5px; }
        .network-item { padding: 10px; border-bottom: 1px solid #eee; cursor: pointer; }
        .network-item:hover { background: #f5f5f5; }
        .network-item:last-child { border-bottom: none; }
        .signal-strength { float: right; font-size: 12px; color: #666; }
        .secure { color: #4CAF50; }
        .info { background: #e3f2fd; padding: 15px; border-radius: 5px; margin-bottom: 20px; color: #1976d2; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🌡️ Smart Cooling System</h1>
        <div class="info">
            <strong>WiFi Configuration</strong><br>
            Current Status: <span id="currentStatus">Loading...</span><br>
            Current Network: <span id="currentNetwork">Loading...</span><br>
            Current Device Token: <span id="currentDeviceToken">Loading...</span><br>
            Connect your ESP32 to your WiFi network to enable remote monitoring and control via ThingsBoard.
        </div>
        
        <form action="/configure" method="post">
            <div class="form-group">
                <label for="ssid">Device Token ID:</label>
                <input type="text" id="deviceToken" name="deviceToken" required placeholder="Enter Your Device Token">
            </div>

            <div class="form-group">
                <label for="ssid">WiFi Network Name (SSID):</label>
                <input type="text" id="ssid" name="ssid" required placeholder="Enter WiFi network name">
            </div>
            
            <div class="form-group">
                <label for="password">WiFi Password:</label>
                <input type="password" id="password" name="password" placeholder="Enter WiFi password (leave empty for open networks)">
            </div>
            
            <button type="submit">💾 Save & Connect</button>
        </form>
        
        <button class="reset-btn" onclick="resetCredentials()">🔄 Reset All Settings</button>
    </div>

    <script>
        function selectNetwork(ssid) {
            document.getElementById('ssid').value = ssid;
            document.getElementById('password').focus();
        }
        
        function resetCredentials() {
            if (confirm('Are you sure you want to reset all WiFi settings? The device will restart.')) {
                window.location.href = '/reset';
            }
        }
        
        // Load current status
        function loadStatus() {
            fetch('/status')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('currentStatus').textContent = data.connected ? 'Connected ✅' : 'Disconnected ❌';
                    document.getElementById('currentNetwork').textContent = data.ssid || 'None';
                    document.getElementById('currentDeviceToken').textContent = data.current_device_token || 'None';
                })
                .catch(error => {
                    document.getElementById('currentStatus').textContent = 'Error loading status';
                    document.getElementById('currentNetwork').textContent = 'Unknown';
                    document.getElementById('currentDeviceToken').textContent = 'Unknown';
                });
        }
        
        // Load status when page loads
        window.onload = function() {
            loadStatus();
        };
    </script>
</body>
</html>
)rawliteral";

    return html;
}

String WiFiManager::generateSuccessPage()
{
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32 Smart Cooling System - Success</title>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial, sans-serif; margin: 0; padding: 20px; background: #f0f2f5; text-align: center; }
        .container { max-width: 500px; margin: 50px auto; background: white; padding: 40px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
        h1 { color: #4CAF50; margin-bottom: 20px; }
        .success-icon { font-size: 64px; margin-bottom: 20px; }
        p { color: #666; font-size: 18px; line-height: 1.6; }
        .countdown { font-size: 24px; color: #4CAF50; font-weight: bold; }
    </style>
</head>
<body>
    <div class="container">
        <div class="success-icon">✅</div>
        <h1>Configuration Saved!</h1>
        <p>Your WiFi credentials have been saved successfully.</p>
        <p>The device will restart and connect to your network.</p>
        <p>After connecting, you can access the system remotely via ThingsBoard.</p>
        <div class="countdown" id="countdown">Restarting in 3 seconds...</div>
    </div>
    
    <script>
        let count = 3;
        setInterval(() => {
            count--;
            if (count > 0) {
                document.getElementById('countdown').textContent = `Restarting in ${count} seconds...`;
            } else {
                document.getElementById('countdown').textContent = 'Restarting now...';
            }
        }, 1000);
    </script>
</body>
</html>
)rawliteral";

    return html;
}

void WiFiManager::startPermanentWebServer()
{
    if (permanentServer != nullptr)
    {
        return; // Already running
    }

    Serial.println("WiFi Manager: Starting permanent web server...");

    permanentServer = new AsyncWebServer(80);

    // Serve the main configuration page
    permanentServer->on("/", HTTP_GET, [this](AsyncWebServerRequest *request)
                        { request->send(200, "text/html", generateConfigPage()); });

    // Handle WiFi configuration submission
    permanentServer->on("/configure", HTTP_POST, [this](AsyncWebServerRequest *request)
                        {
        String newSsid = "";
        String newPassword = "";
        String newDeviceToken = "";
        
        if (request->hasParam("ssid", true))
        {
            newSsid = request->getParam("ssid", true)->value();
        }
        
        if (request->hasParam("password", true))
        {
            newPassword = request->getParam("password", true)->value();
        }

        if (request->hasParam("deviceToken", true))
        {
            newDeviceToken = request->getParam("deviceToken", true)->value();
        }
        
        if (newSsid.length() > 0 && newDeviceToken.length() > 0)
        {
            saveCredentials(newSsid, newPassword, newDeviceToken);
            request->send(200, "text/html", generateSuccessPage());
            
            // Delay and restart to apply new settings
            delay(3000);
            ESP.restart();
        }
        else
        {
            request->send(400, "text/html", "<h1>Error: SSID cannot be empty!</h1><a href='/'>Go back</a>");
        } });

    // Handle reset credentials
    permanentServer->on("/reset", HTTP_GET, [this](AsyncWebServerRequest *request)
                        {
        preferences.clear();
        ssid = "";
        password = "";
        deviceToken = "";
        request->send(200, "text/html", "<h1>Credentials Reset!</h1><p>Device will restart...</p>");
        delay(3000);
        ESP.restart(); });

    // Serve scan results
    permanentServer->on("/scan", HTTP_GET, [this](AsyncWebServerRequest *request)
                        {
        String json = "[";
        int n = WiFi.scanNetworks();
        for (int i = 0; i < n; ++i)
        {
            if (i) json += ",";
            json += "{";
            json += "\"ssid\":\"" + WiFi.SSID(i) + "\",";
            json += "\"rssi\":" + String(WiFi.RSSI(i)) + ",";
            json += "\"secure\":" + String(WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
            json += "}";
        }
        json += "]";
        request->send(200, "application/json", json); });

    // Add a status endpoint
    permanentServer->on("/status", HTTP_GET, [this](AsyncWebServerRequest *request)
                        {
        String json = "{";
        json += "\"current_device_token\": \"" + deviceToken + "\",";
        json += "\"connected\": " + String(isConnected() ? "true" : "false") + ",";
        json += "\"ssid\": \"" + ssid + "\",";
        json += "\"ip\": \"" + getLocalIP().toString() + "\",";
        json += "\"rssi\": " + String(WiFi.RSSI()) + ",";
        json += "\"uptime\": " + String(millis());
        json += "}";
        request->send(200, "application/json", json); });

    permanentServer->begin();
    Serial.println("WiFi Manager: Permanent web server started on port 80");
    Serial.print("WiFi Manager: Access at http://");
    Serial.println(getLocalIP());

    // Update current IP for telemetry
    strcpy(current_ip, getLocalIP().toString().c_str());
}

void WiFiManager::stopPermanentWebServer()
{
    if (permanentServer != nullptr)
    {
        Serial.println("WiFi Manager: Stopping permanent web server...");
        permanentServer->end();
        delete permanentServer;
        permanentServer = nullptr;
    }
}

const char *WiFiManager::getCurrentIP()
{
    // Update current IP and return it
    strcpy(current_ip, getLocalIP().toString().c_str());
    return current_ip;
}

void WiFiManager::handleWebServer()
{
    // This method is called from the main loop to handle web server requests
    // AsyncWebServer handles requests automatically, so this is mostly a placeholder
    // for future functionality if needed
}