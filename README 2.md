# MetaOSC

A C++ application that bridges MetaMotion sensor data to OSC (Open Sound Control) for real-time motion tracking applications.

## Overview

MetaOSC connects to MetaMotion sensors via Bluetooth LE and streams sensor data (orientation, acceleration, magnetometer, and gyroscope) to one or more OSC servers. This enables real-time motion tracking for creative applications, interactive installations, performance art, and more.

## Features

- **Multiple Sensor Support**: Connect to multiple MetaMotion sensors simultaneously
- **Multiple OSC Destinations**: Stream data to multiple OSC servers concurrently
- **Comprehensive Sensor Data**:
  - Euler angles (quaternion orientation): `/euler/{index} w x y z`
  - Acceleration: `/acc/{index} x y z`
  - Magnetometer: `/mag/{index} x y z`
  - Gyroscope: `/gyro/{index} x y z`
- **Configurable**: JSON-based configuration for MAC address filtering and OSC server endpoints
- **Graceful Shutdown**: Proper handling of SIGINT (Ctrl+C) with clean disconnection

## Requirements

- CMake 3.1 or higher
- C++17 compatible compiler
- [JUCE](https://juce.com/) framework
- [SimpleBLE](https://github.com/OpenBluetoothToolbox/SimpleBLE) library
- [nlohmann/json](https://github.com/nlohmann/json) library
- MetaMotion sensors (MetaWear boards)

## Building

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

## Configuration

### Default Configuration

By default, MetaOSC will:
- Connect to **all** available MetaMotion sensors
- Send OSC data to:
  - `127.0.0.1:8000`
  - `127.0.0.1:8001`

### Custom Configuration

Create a JSON configuration file to customize behavior:

```json
{
  "macs": [
    "AA:BB:CC:DD:EE:FF",
    "11:22:33:44:55:66"
  ],
  "servers": [
    {
      "host": "192.168.1.100",
      "port": 9000
    },
    {
      "host": "127.0.0.1",
      "port": 8000
    }
  ]
}
```

**Configuration Options:**
- `macs`: Array of MAC addresses to filter which sensors to connect to. Leave empty `[]` to connect to all available MetaMotion sensors.
- `servers`: Array of OSC server endpoints to send data to.

### Running with Configuration

```bash
./MetaOSC --config path/to/config.json
# or
./MetaOSC -c path/to/config.json
```

## OSC Message Format

MetaOSC sends OSC messages at approximately 10Hz (100ms intervals). Each sensor is identified by an index (starting at 0).

### Message Types

| OSC Address | Arguments | Description |
|------------|-----------|-------------|
| `/euler/{index}` | `w x y z` | Quaternion orientation (Euler angles) |
| `/acc/{index}` | `x y z` | Linear acceleration in m/s² |
| `/mag/{index}` | `x y z` | Magnetometer readings |
| `/gyro/{index}` | `x y z` | Gyroscope readings in rad/s |

**Example:**
```
/euler/0 1.0 0.0 0.0 0.0
/acc/0 0.1 -9.8 0.2
/mag/0 25.3 -12.1 48.7
/gyro/0 0.05 -0.02 0.01
```

## Usage Example

1. **Start MetaOSC** with your configuration:
   ```bash
   ./MetaOSC -c my_config.json
   ```

2. **Receive OSC data** in your application (e.g., TouchDesigner, Max/MSP, Processing, Unity, etc.)

3. **Stop gracefully** by pressing `Ctrl+C`

## Logging

MetaOSC logs operational information to the console, including:
- Startup and shutdown events
- Configuration loading
- Sensor connection status
- Real-time Euler angle data (for monitoring)

## Troubleshooting

### No sensors found
- Ensure MetaMotion sensors are powered on and in range
- Check Bluetooth is enabled on your system
- Verify MAC addresses in config file are correct (if filtering)

### OSC data not received
- Verify the receiving application is listening on the correct port
- Check firewall settings
- Ensure the host IP address is correct in the configuration

### Connection issues
- The application waits 2 seconds after scanning before connecting
- Each sensor connection has a 2-second delay to ensure stability
- Try reducing the number of simultaneous sensor connections

## Architecture

- **BleInterface**: Manages Bluetooth Low Energy scanning and device discovery
- **MetaMotionController**: Handles individual sensor connections and data streaming
- **MetaOSCThread**: Main thread that coordinates data collection and OSC transmission
- **JUCE OSCSender**: Provides OSC protocol implementation

## License

[Add your license information here]

## Contributing

[Add contribution guidelines here]

## Acknowledgments

- Built with [JUCE](https://juce.com/) audio framework
- Uses [SimpleBLE](https://github.com/OpenBluetoothToolbox/SimpleBLE) for Bluetooth connectivity
- Uses [nlohmann/json](https://github.com/nlohmann/json) for configuration parsing
- Compatible with [MetaWear](https://mbientlab.com/) sensors
