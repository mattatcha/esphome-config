# Navitas TAC Controller ESPHome Setup Guide

This ESPHome configuration allows you to read all supported parameters from a Navitas TAC golf cart motor controller via Bluetooth Low Energy (BLE).

## Prerequisites

- ESP32 development board with BLE support
- ESPHome installed and configured
- Home Assistant (optional, for integration)
- Navitas TAC-equipped golf cart
- BLE scanner app for discovery

## Hardware Setup

1. **ESP32 Board**: Use any ESP32 board with BLE support (ESP32-DevKitC, ESP32-WROOM, etc.)
2. **Power**: The ESP32 can be powered via USB or external 5V supply
3. **Mounting**: Mount the ESP32 somewhere on your golf cart with good BLE range to the controller

## Software Setup

### Step 1: Discover Your TAC Controller

Before using this configuration, you need to discover the actual BLE parameters of your TAC controller:

1. **Find the MAC Address**:
   - Use a BLE scanner app (nRF Connect, BLE Scanner, etc.) 
   - Look for a device near your golf cart (may be named "Navitas", "TAC", or show as unknown)
   - Note the MAC address (format: AA:BB:CC:DD:EE:FF)

2. **Discover BLE Services and Characteristics**:
   - Connect to the device in your BLE scanner app
   - Browse the available services and characteristics
   - Note down the UUIDs for:
     - Main service UUID
     - Read characteristic UUID  
     - Write characteristic UUID
     - Notify characteristic UUID (for real-time updates)

### Step 2: Configure ESPHome

1. **Update secrets.yaml**:
   - Copy `secrets.yaml` to your ESPHome config directory
   - Update all values with your specific information:
     - WiFi credentials
     - TAC controller MAC address
     - BLE service/characteristic UUIDs
     - Golf cart configuration (tire size, gear ratio)

2. **Update the YAML Configuration**:
   - Replace placeholder UUIDs in `navitas_tac_controller.yaml`
   - Adjust the `mac_address` in the `ble_client` section
   - Customize sensor names and update intervals as needed

### Step 3: Parameters Available

Based on the TAC dictionary, this configuration reads:

**Core Measurements**:
- Controller Temperature (°C)
- DC Bus Voltage (V) 
- Battery Current (A)
- Motor Phase Currents (Ia, Ib, Ic) 
- Logic Power Voltage (V)
- Motor RPM

**Calculated Values**:
- Vehicle Speed (mph/kph) - calculated from RPM, tire diameter, gear ratio
- Drive Mode (Forward/Reverse/Neutral)

**Status Indicators**:
- Forward/Reverse/Key Switch states
- Neutral detection
- Fault conditions (Group 1 faults)

**Configuration Parameters**:
- Tire Diameter (Flash memory)
- Rear Axle Ratio

### Step 4: Protocol Implementation

The TAC controller uses a JSON-based command protocol over BLE:

**Read Command**:
```json
{
  "command": "Read",
  "address": 38,
  "memory_type": 0
}
```

**Write Command**:
```json
{
  "command": "Write", 
  "address": 155,
  "value": 1280,
  "memory_type": 1
}
```

**Memory Types**:
- 0: ReadOrWrite (RAM parameters)
- 1: Flash (configuration parameters)
- 2: FlashReadOnly (read-only config)

### Step 5: Deploy and Test

1. **Flash the ESP32**:
   ```bash
   esphome run navitas_tac_controller.yaml
   ```

2. **Monitor Logs**:
   ```bash
   esphome logs navitas_tac_controller.yaml
   ```

3. **Check Connection**:
   - Look for "Connected to Navitas TAC Controller" in logs
   - Verify sensor values are updating in Home Assistant

## Troubleshooting

### Connection Issues
- Ensure the MAC address is correct
- Check BLE range (ESP32 should be within ~10 meters of controller)
- Verify the golf cart controller is powered on
- Try power cycling both the ESP32 and golf cart

### Missing Data
- Confirm BLE service/characteristic UUIDs are correct
- Check if the controller requires pairing/authentication
- Monitor ESPHome logs for BLE communication errors
- Some parameters may only be available when the golf cart is running

### Parameter Scaling
- If values seem wrong, verify the scale factors from TACDictionary.js
- Some parameters may be signed integers (RPM can be negative for reverse)
- Temperature and voltage readings should be reasonable ranges

## Advanced Configuration

### Adding More Parameters

To add additional TAC parameters:

1. Find the parameter in `TACDictionary.js`
2. Note the Address, Scale, and MemoryType
3. Add a new sensor in the YAML with the correct scaling formula
4. Update the C++ component if needed for complex calculations

### Custom Calculations

The configuration includes examples of:
- Speed calculation from RPM (NavitasMotorControllerModel.js:726-742)
- Switch bit decoding (NavitasMotorControllerModel.js:687-724)
- Signed integer handling for RPM values

### Writing Parameters

To write configuration parameters (like tire diameter):
- Use the `number` platform with appropriate constraints
- Ensure you're writing to Flash memory type for persistent settings
- Be careful with parameter ranges to avoid damaging the controller

## Safety Notes

- This is for monitoring only - avoid writing critical control parameters
- Test thoroughly before relying on this data for operational decisions
- The golf cart controller has built-in safety systems that this setup does not override
- Always follow proper golf cart safety procedures

## Integration with Home Assistant

Once configured, all sensors will automatically appear in Home Assistant:
- Create dashboards showing battery status, speed, temperature
- Set up automations based on controller states
- Monitor fault conditions and receive alerts
- Log data for analysis and maintenance planning

## Support

This configuration is based on reverse-engineering the official Navitas app. If you encounter issues:

1. Check the ESPHome logs for detailed error messages
2. Verify BLE communication with standard BLE tools
3. Compare parameter addresses with the TACDictionary.js file
4. Consider that different TAC firmware versions may have slight differences