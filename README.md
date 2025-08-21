# ESP32 Smart Cooling System

An advanced IoT-enabled temperature control system using ESP32, designed to maintain precise temperature control of a Peltier cooling chip with remote monitoring and control capabilities via ThingsBoard.

## 🌟 Features

- **Precise Temperature Control**: PID algorithm maintains target temperature within ±0.1°C
- **IoT Integration**: Real-time monitoring and control via ThingsBoard platform
- **WiFi Management**: Easy network configuration with web-based setup portal
- **Modular Architecture**: Clean, maintainable code with separate classes for each component
- **Real-time Operation**: FreeRTOS multitasking for responsive control
- **Remote Control**: Change target temperature and PID parameters remotely
- **Safety Features**: Temperature limits, sensor failure detection, and automatic shutdown
- **Comprehensive Telemetry**: Real-time data streaming for monitoring and analysis

## 🏗️ System Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   WiFi Manager  │    │ Temperature      │    │  PID Controller │
│   - Web Portal  │    │ Sensor (DS18B20) │    │  - PWM Output   │
│   - Auto-Connect│    │ - Real-time Read │    │  - MOSFET Drive │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         │                        │                       │
         └────────────────────────┼───────────────────────┘
                                  │
              ┌─────────────────────────────────────┐
              │         Main Controller            │
              │    - FreeRTOS Coordination         │
              │    - Thread-safe Communication     │
              │    - System Initialization         │
              └─────────────────────────────────────┘
                                  │
                    ┌─────────────────────────┐
                    │  ThingsBoard Client     │
                    │  - MQTT Communication  │
                    │  - Telemetry Streaming │
                    │  - RPC Commands         │
                    └─────────────────────────┘
```

## 🔧 Hardware Requirements

### Core Components
- **ESP32 Development Board** (ESP32-DOIT-DEVKIT-V1 or compatible)
- **DS18B20 Temperature Sensor** (waterproof version recommended)
- **MOSFET PWM Module** (for 12V Peltier control)
- **12V Peltier Cooling Chip** (TEC1-12706 or similar)
- **4.7kΩ Pull-up Resistor** (for DS18B20 data line)

### Power Supply
- **12V DC Power Supply** (minimum 6A for Peltier operation)
- **ESP32 Power** (via USB or separate 5V supply)

### Optional Components
- Heat sink and thermal paste for Peltier chip
- Fan for improved heat dissipation
- Enclosure for protection

## 🔌 Pin Configuration

| Component | ESP32 Pin | Description |
|-----------|-----------|-------------|
| DS18B20 Data | GPIO2 (D2) | Temperature sensor data line |
| PWM Output | GPIO15 (D15) | MOSFET control signal |
| Power | 3.3V/5V | Sensor power supply |
| Ground | GND | Common ground |

### DS18B20 Wiring
```
DS18B20          ESP32
VCC     ────────  3.3V
GND     ────────  GND
DATA    ────┬───  GPIO2 (D2)
            │
        4.7kΩ (pull-up to 3.3V)
```

## 📚 Software Dependencies

The project uses PlatformIO with the following libraries:

```ini
lib_deps = 
    paulstoffregen/OneWire@^2.3.8
    milesburton/DallasTemperature@^4.0.4
    thingsboard/ThingsBoard@^0.15.0
    https://github.com/me-no-dev/AsyncTCP.git
    https://github.com/me-no-dev/ESPAsyncWebServer.git
```

## 🚀 Quick Start

### 1. Hardware Setup
1. Connect the DS18B20 sensor to GPIO2 with 4.7kΩ pull-up resistor
2. Connect the MOSFET module input to GPIO15
3. Wire the Peltier chip through the MOSFET module
4. Apply thermal paste and attach heat sink to Peltier chip

### 2. Software Installation
1. Install [PlatformIO](https://platformio.org/) IDE or extension
2. Clone this repository:
   ```bash
   git clone <repository-url>
   cd ant_box_test
   ```
3. Open the project in PlatformIO
4. Upload the code to your ESP32:
   ```bash
   pio run --target upload
   ```

### 3. WiFi Configuration
1. On first boot, ESP32 creates WiFi access point: `ESP32-SmartCooling`
2. Connect to this AP with password: `12345678`
3. Open browser and navigate to `192.168.4.1`
4. Select your WiFi network and enter credentials
5. Click "Save & Connect" - device will restart and connect to your network

### 4. ThingsBoard Setup
1. Create account on [ThingsBoard Cloud](https://thingsboard.cloud/) or [Demo Server](https://demo.thingsboard.io/)
2. Create new device and copy the access token
3. Update `TOKEN` in `include/config.h` with your device token
4. Recompile and upload the modified code

## ⚙️ Configuration

### Temperature Settings
Default configuration in `include/config.h`:
- **Target Temperature**: 25.0°C
- **PWM Frequency**: 10 Hz
- **Safety Limits**: -10°C to 60°C
- **Reading Interval**: 500ms

### PID Parameters
Default PID settings (tuned for typical Peltier cooling):
- **Kp (Proportional)**: 50.0
- **Ki (Integral)**: 1.5
- **Kd (Derivative)**: 12.0

### ThingsBoard Configuration
```cpp
// Update these in config.h
constexpr char TOKEN[] = "your_device_token_here";
constexpr char THINGSBOARD_SERVER[] = "demo.thingsboard.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;
```

## 📊 Monitoring & Control

### ThingsBoard Dashboard

The system automatically sends telemetry data every 2 seconds:

| Parameter | Description | Unit |
|-----------|-------------|------|
| `temperature` | Current temperature reading | °C |
| `targetTemp` | Target temperature setpoint | °C |
| `pwmValue` | Raw PWM output (0-1023) | - |
| `pwmPercent` | PWM output percentage | % |
| `error` | Temperature error (current - target) | °C |
| `pidOutput` | PID controller output | - |
| `sensorConnected` | Sensor connection status | boolean |
| `kp`, `ki`, `kd` | Current PID parameters | - |

### Remote Control (RPC Commands)

#### Change Target Temperature
```json
{
  "method": "setTargetTemp",
  "params": {
    "value": 23.5
  }
}
```

#### Update PID Parameters
Put this into Terminal
```bash
setPidParams "{\"kp\": 45, \"ki\": 1.5, \"kd\": 10}"
```

## 🔧 Advanced Usage

### Serial Monitor Output
Connect to serial monitor (115200 baud) to view:
- System initialization status
- Real-time temperature readings
- PID calculations
- WiFi connection status
- ThingsBoard communication logs

### PID Tuning
For optimal performance, tune PID parameters based on your specific setup:

1. **Start with P-only control**: Set Ki=0, Kd=0, adjust Kp until system responds
2. **Add Integral**: Gradually increase Ki to eliminate steady-state error
3. **Add Derivative**: Increase Kd to reduce overshoot and improve stability

### Safety Features
- **Temperature Limits**: System shuts down if temperature exceeds safe range
- **Sensor Failure**: Cooling disabled if sensor disconnects
- **Large Error Protection**: Maximum cooling applied for large temperature differences
- **Integral Windup Protection**: Prevents integral term from growing excessively

## 🛠️ Troubleshooting

### Common Issues

**1. Temperature Sensor Not Found**
- Check wiring and 4.7kΩ pull-up resistor
- Verify sensor is DS18B20 and not damaged
- Test with simple OneWire scanner sketch

**2. WiFi Connection Issues**
- Reset WiFi credentials via web portal
- Check network name and password
- Ensure 2.4GHz network (ESP32 doesn't support 5GHz)

**3. ThingsBoard Connection Failed**
- Verify device token is correct
- Check internet connectivity
- Ensure firewall allows MQTT (port 1883)

**4. Poor Temperature Control**
- Check thermal interface between sensor and cooling surface
- Verify Peltier chip polarity and power supply
- Tune PID parameters for your specific setup

### Diagnostic Commands

Enable debug output by uncommenting in `main.cpp`:
```cpp
// pidController.outputSerialData(currentTemp);
```

## 📈 Performance Characteristics

- **Temperature Accuracy**: ±0.1°C (with proper thermal coupling)
- **Response Time**: ~30-60 seconds (depends on thermal mass)
- **Power Consumption**: 
  - ESP32: ~250mA @ 3.3V
  - Peltier: Up to 6A @ 12V (variable with PWM)
- **Communication Latency**: <100ms for RPC commands
- **Telemetry Rate**: 0.5Hz (configurable)

## 🔄 System Tasks

The system runs multiple FreeRTOS tasks for optimal performance:

| Task | Core | Priority | Function |
|------|------|----------|----------|
| Temperature Reading | 0 | High (3) | DS18B20 sensor polling |
| PID Control | 1 | Medium (2) | Temperature control calculations |
| ThingsBoard Comm | 0 | Low (1) | IoT communication |
| Coordination | 1 | Medium (2) | System orchestration |

## 🏷️ Version History

- **v1.0.0**: Initial release with basic PID control
- **v1.1.0**: Added ThingsBoard integration
- **v1.2.0**: WiFi manager with web portal
- **v1.3.0**: FreeRTOS multitasking implementation
- **v1.4.0**: Enhanced safety features and RPC control

## 📄 License

This project is open source. Feel free to modify and distribute according to your needs.

## 🤝 Contributing

Contributions are welcome! Areas for improvement:
- Web dashboard for local monitoring
- Data logging to SD card
- Multiple sensor support
- Advanced control algorithms
- Mobile app integration

## 📞 Support

For issues and questions:
1. Check the troubleshooting section
2. Review serial monitor output
3. Verify hardware connections
4. Test with minimal configuration

---

**⚠️ Safety Warning**: This system controls heating/cooling devices. Always implement appropriate safety measures, monitor operation, and ensure proper ventilation. The developers are not responsible for any damage or injury resulting from use of this system. 