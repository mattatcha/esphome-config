# Navitas TAC ESPHome Component

An ESPHome component for communicating with Navitas TAC golf cart motor controllers via Bluetooth Low Energy (BLE).

## Overview

This component enables real-time monitoring of Navitas TAC motor controllers, providing access to critical operational data including temperatures, voltages, currents, RPM, calculated vehicle speed, and gear state. The implementation is based on reverse-engineering the official Navitas mobile application and TAC protocol documentation.

## Features

### Sensors
- **Controller Temperature** - TAC controller internal temperature (°C)
- **Motor Temperature** - Electric motor temperature (°C)
- **DC Bus Voltage** - Main battery voltage (V)
- **Battery Current** - Current draw from battery (A)
- **Motor RPM** - Motor rotational speed (signed: positive=forward, negative=reverse)
- **Vehicle Speed** - Calculated speed from RPM, tire diameter, and gear ratio (mph/km/h)
- **State of Charge** - Battery state of charge percentage (%)

### Text Sensors
- **Connection State** - Connected/Disconnected status
- **Gear State** - Forward/Reverse/Neutral based on switch positions

### Binary Sensors
- **Connected** - BLE connection status

## Hardware Requirements

- ESP32 development board with BLE support
- Navitas TAC-equipped golf cart
- BLE range: typically 10-30 meters depending on environment

## Installation

1. Copy the `navitas_tac` folder to your ESPHome `components` directory
2. Configure your YAML as shown below
3. Flash to your ESP32

## Configuration

### Basic Configuration

```yaml
external_components:
  - source: path/to/components
    components: [navitas_tac]

esp32_ble_tracker:

ble_client:
  - mac_address: "AA:BB:CC:DD:EE:FF"  # Replace with your TAC controller MAC
    id: navitas_tac_ble_client

navitas_tac:
  ble_client_id: navitas_tac_ble_client
  id: tac_controller
  update_interval: 5s

  # Vehicle configuration (fallback values if not available from controller)
  tire_diameter: 22.0      # inches - fallback if controller config unavailable
  rear_axle_ratio: 12.5    # gear ratio - fallback if controller config unavailable
  use_kilometers: false    # units - fallback if controller config unavailable

sensor:
  - platform: navitas_tac
    navitas_tac_id: tac_controller
    controller_temperature:
      name: "Controller Temperature"
    motor_temperature:
      name: "Motor Temperature"
    dc_bus_voltage:
      name: "DC Bus Voltage"
    battery_current:
      name: "Battery Current"
    motor_rpm:
      name: "Motor RPM"
    speed:
      name: "Vehicle Speed"
    soc:
      name: "State of Charge"

text_sensor:
  - platform: navitas_tac
    navitas_tac_id: tac_controller
    state:
      name: "Controller State"
    gear:
      name: "Vehicle Gear"

binary_sensor:
  - platform: navitas_tac
    navitas_tac_id: tac_controller
    connected:
      name: "Controller Connected"
```

## Configuration Options

### Main Component (`navitas_tac`)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `ble_client_id` | string | **Required** | ID of the BLE client component |
| `update_interval` | time | `2s` | How often to request data from controller |
| `tire_diameter` | float | `22.0` | Fallback tire diameter in inches (10.0-50.0) |
| `rear_axle_ratio` | float | `12.5` | Fallback rear axle gear ratio (1.0-50.0) |
| `use_kilometers` | boolean | `false` | Fallback units - controller setting takes priority |

### Sensors

All sensors are optional. Include only the ones you need:

- `controller_temperature` - Controller internal temperature
- `motor_temperature` - Motor temperature
- `dc_bus_voltage` - Battery voltage
- `battery_current` - Current consumption
- `motor_rpm` - Motor rotational speed
- `speed` - Calculated vehicle speed
- `soc` - State of charge (if supported by controller)

### Text Sensors

- `state` - Connection status text
- `gear` - Current gear (Forward/Reverse/Neutral)

### Binary Sensors

- `connected` - BLE connection status

## Speed Calculation

Vehicle speed is calculated using the formula from the official Navitas app:

```
speed = (|RPM| × tire_diameter ÷ 2) ÷ gear_ratio × conversion_factor
```

**Parameter Sources:**
- **RPM**: Read from controller motor sensor (signed: negative=reverse, positive=forward)
- **Tire diameter & gear ratio**: Automatically retrieved from controller configuration
- **Units (mph/km/h)**: Retrieved from controller settings
- **Fallback values**: Uses configured defaults if controller values unavailable

**Notes:**
- Speed magnitude is always positive; direction indicated by gear sensor
- Controller configuration takes priority over YAML defaults
- Vehicle config is read once at startup and cached for performance

## Finding Your Controller

Before configuring the component, you need to discover your TAC controller's BLE MAC address using ESPHome.

### ESPHome Discovery Method (Recommended)

Create a discovery configuration to scan and identify your TAC controller:

```yaml
esp32_ble_tracker:
  scan_parameters:
    duration: 300s  # Scan for 5 minutes
    active: true
  on_ble_advertise:
    then:
      - lambda: |-
          // Log all devices for analysis
          ESP_LOGI("ble_discovery", "Found device: %s", x.address_str().c_str());

          // Check for Navitas TAC service UUID (0xFFB0)
          for (auto service : x.get_service_uuids()) {
            if (service.get_uuid().uuid.uuid16 == 0xFFB0) {
              ESP_LOGW("navitas_tac", "*** FOUND TAC CONTROLLER: %s ***", x.address_str().c_str());
              if (!x.get_name().empty()) {
                ESP_LOGW("navitas_tac", "    Name: %s", x.get_name().c_str());
              }
            }
          }

logger:
  level: INFO
```

**Steps:**
1. **Flash this discovery configuration** to your ESP32
2. **Power on your golf cart** - The TAC controller must be active
3. **Check the ESPHome logs** - Look for messages starting with `*** FOUND TAC CONTROLLER:`
4. **Copy the MAC address** from the log output (format: `AA:BB:CC:DD:EE:FF`)

### Alternative: Mobile App Discovery

If ESPHome discovery doesn't work, use a BLE scanner app as backup:
- **nRF Connect** (Nordic Semiconductor) - iOS/Android
- Look for devices with service UUID `FFB0`
- Note the MAC address

## Troubleshooting

### Connection Issues
- **Check MAC address** - Ensure it matches your specific controller
- **BLE range** - Keep ESP32 within 10-30 meters of controller
- **Controller power** - TAC must be powered on and operational
- **Interference** - Minimize WiFi/BLE interference

### Data Issues
- **Missing values** - Some parameters only available when cart is running
- **Wrong scaling** - Check tire diameter and gear ratio settings
- **Negative RPM** - Normal in reverse; speed shows absolute value

### Common Error Messages
- `"Not connected to TAC controller"` - BLE connection lost
- `"Required BLE characteristics not found"` - Wrong device or incompatible firmware
- `"Failed to enable notifications"` - BLE communication error

## Technical Documentation

For detailed protocol documentation, parameter mappings, and implementation details, see [PROTOCOL.md](PROTOCOL.md).

## Safety Considerations

- **Monitoring only** - This component reads data; it does not control the cart
- **No safety override** - Golf cart's built-in safety systems remain active
- **Data accuracy** - Verify critical readings before operational decisions
- **Vehicle safety** - Always follow proper golf cart safety procedures

## Home Assistant Integration

All sensors automatically appear in Home Assistant with appropriate device classes:
- Temperature sensors show as temperature entities
- Current/voltage sensors use energy device classes
- Speed sensor uses speed device class
- Gear sensor appears as text entity


## Compatibility

### Tested Controllers
- Navitas TAC 2.0 series
- Various golf cart manufacturers using Navitas controllers

### ESPHome Versions
- ESPHome 2025.8.0+ (latest stable version)
- ESP32 with ESP-IDF framework recommended

## Support

This component is based on reverse-engineering the official Navitas mobile application. For issues:

1. Check ESPHome logs for detailed error messages
2. Verify BLE communication with standard tools
3. Ensure controller firmware compatibility
4. Different TAC versions may have slight protocol differences

## License

This component is provided as-is for educational and personal use. Navitas TAC is a trademark of Navitas Vehicle Systems.
