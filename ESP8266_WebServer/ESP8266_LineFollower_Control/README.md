# ESP8266 Line Follower Robot Control

This ESP8266 project provides a web interface for remote control of your STM32 line follower robot with its own WiFi access point.

## Features

- **Own WiFi Access Point**: No need to connect to existing WiFi
- **Web Interface**: Control your robot from any device with a web browser
- **EEPROM Storage**: Settings persist between reboots
- **Auto-Configure STM32**: Sends saved settings to STM32 on boot
- **Real-time Monitoring**: Live sensor data and PID values
- **PID Tuning**: Adjust Kp, Ki, Kd, base speed, and thrust from the web interface
- **Motor Control**: Direct control of DC motors and ESC thrusters
- **Line Following Control**: Start/stop line following and sweeping modes
- **Mobile Responsive**: Works on phones, tablets, and computers

## Hardware Setup

### ESP8266 Connections
- **TX (GPIO1)**: Connect to STM32 RX (USART1)
- **RX (GPIO3)**: Connect to STM32 TX (USART1)
- **GND**: Connect to STM32 GND
- **3.3V**: Power from 3.3V supply (or USB)

### STM32 Connections
- **USART1 TX**: Connect to ESP8266 RX
- **USART1 RX**: Connect to ESP8266 TX
- **GND**: Connect to ESP8266 GND

## Software Setup

### 1. Install Required Libraries
In Arduino IDE, install these libraries:
- ESP8266WiFi
- ESP8266WebServer
- ESP8266mDNS
- ArduinoJson

### 2. Upload Code
- Select ESP8266 board in Arduino IDE
- Upload the code to your ESP8266

## Usage

### 1. Access Web Interface
- Power on your ESP8266 (LED will turn on when ready)
- Connect your device to WiFi network: **"LineFollower_Robot"**
- Password: **"robot123"**
- Open web browser and go to: **http://192.168.4.1**

### 2. Control Commands
- **Line Following**: Start/stop autonomous line following
- **PID Tuning**: Adjust Kp, Ki, Kd values in real-time
- **Motor Control**: Direct control of DC motors and ESC thrusters
- **Status Monitoring**: Real-time view of all sensor and motor data

### 3. Persistent Settings
- **Auto-Save**: PID values are automatically saved to EEPROM when changed
- **Auto-Restore**: Settings are restored from EEPROM on power-up
- **Auto-Configure**: ESP8266 sends all saved settings to STM32 on boot
- **No Setup Required**: STM32 gets the correct settings automatically

## Web Interface Features

### Status Display
- Robot mode indicators (Line Following, Sweeping)
- 7-sensor array visualization
- Line position display
- Real-time motor speeds and ESC thrust

### Control Panel
- One-click line following start/stop
- PID parameter sliders
- Motor speed controls
- Emergency stop buttons

### Data Monitoring
- Live sensor values
- PID output values
- Motor speeds and directions
- ESC thrust levels

## Troubleshooting

### Common Issues
1. **Can't Connect to WiFi**
   - Look for "LineFollower_Robot" network
   - Use password "robot123"
   - Make sure ESP8266 LED is on (indicates AP is ready)

2. **Serial Communication Issues**
   - Verify TX/RX connections
   - Check baud rate (115200)
   - Ensure STM32 is powered

3. **Web Interface Not Loading**
   - Go to exactly: http://192.168.4.1
   - Make sure you're connected to "LineFollower_Robot" WiFi
   - Try refreshing browser

### Serial Monitor Output
The ESP8266 provides detailed status via serial monitor:
- WiFi connection status
- IP address assignment
- Web server status
- Serial communication with STM32

## Customization

### Adding New Commands
1. Add command handler in `handleCommand()`
2. Update web interface HTML
3. Add JavaScript function for new command

### Modifying PID Parameters
1. Adjust default values in variables
2. Update web interface ranges
3. Modify parsing functions if needed

### Adding New Sensors
1. Extend sensor arrays
2. Update parsing functions
3. Modify web interface display

## Technical Details

- **Access Point**: Creates "LineFollower_Robot" WiFi network
- **IP Address**: 192.168.4.1 (fixed)
- **Web Server**: ESP8266WebServer on port 80
- **Serial Baud**: 115200 bps
- **Update Rate**: 1Hz (1000ms intervals)
- **EEPROM Storage**: Settings saved automatically
- **Auto-Boot Config**: Sends settings to STM32 on startup

## Support

For issues or questions:
1. Make sure ESP8266 LED is on (Access Point ready)
2. Connect to "LineFollower_Robot" WiFi with password "robot123"
3. Go to http://192.168.4.1 in browser
4. Verify hardware connections (TX/RX between ESP8266 and STM32)
5. Ensure STM32 is powered and responding to serial commands

## First Time Setup

1. **Upload Code**: Flash the ESP8266 with the provided code
2. **Power On**: ESP8266 LED will turn on when Access Point is ready
3. **Connect**: Join "LineFollower_Robot" WiFi network (password: robot123)
4. **Access**: Open http://192.168.4.1 in your browser
5. **Configure**: Adjust PID values as needed - they'll be saved automatically!
6. **Use**: Control your robot wirelessly without any laptop connection

## License

This project is open source and available under the MIT License.
