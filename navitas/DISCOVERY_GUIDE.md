# Navitas TAC Controller Auto-Discovery Guide

This guide explains how to automatically discover your Navitas TAC controller's MAC address and BLE service/characteristic UUIDs using ESPHome.

## Overview

I've created three ESPHome configurations to help with TAC controller discovery:

1. **`navitas_tac_discovery.yaml`** - Basic discovery scanner
2. **`navitas_tac_auto_config.yaml`** - Smart auto-discovery with confidence scoring
3. **`navitas_tac_controller.yaml`** - Main configuration for reading TAC parameters

## Step 1: Basic Discovery

### Flash the Discovery Configuration

1. **Prepare your ESP32:**
   ```bash
   esphome run navitas_tac_discovery.yaml
   ```

2. **Monitor the logs:**
   ```bash
   esphome logs navitas_tac_discovery.yaml
   ```

3. **Position near golf cart:**
   - Ensure golf cart is powered on
   - Place ESP32 within 10 meters of the TAC controller
   - Look for log messages starting with "*** POTENTIAL TAC CONTROLLER FOUND ***"

### What to Look For

The discovery system searches for devices with:
- **Names containing:** navitas, tac, golf, cart, motor, controller
- **Strong signal strength** (RSSI > -60 dB) for unnamed devices
- **Multiple BLE services** indicating a complex controller device

## Step 2: Smart Auto-Discovery

### Use the Auto-Config Version

The `navitas_tac_auto_config.yaml` provides:
- **Confidence scoring** for potential TAC devices
- **Automatic best candidate selection**
- **Real-time discovery status in Home Assistant**
- **One-click configuration generation**

### Home Assistant Integration

Once flashed, you'll see these entities in Home Assistant:
- **Discovered TAC MAC** - Shows found MAC address
- **Discovered TAC Name** - Shows device name
- **Discovery Confidence** - Confidence percentage (0-100%)
- **TAC Signal Strength** - RSSI in dB

### Manual Controls

- **Connect to TAC** button - Attempt connection to discovered device
- **Restart Discovery** button - Clear results and scan again  
- **Generate Config** button - Output configuration for main setup
- **Auto Discovery** switch - Enable/disable continuous scanning

## Step 3: Service and Characteristic Discovery

### Method 1: Using nRF Connect (Recommended)

1. **Install nRF Connect for Mobile** (iOS/Android)
2. **Scan for devices** near your golf cart
3. **Connect to the discovered MAC address**
4. **Browse Services:**
   - Look for services with multiple characteristics
   - Note services with Read, Write, and Notify properties
   - Services typically have UUIDs like: `12345678-1234-1234-1234-123456789ABC`

5. **Test characteristics:**
   - Try reading from different characteristics
   - Look for ones that return data (not errors)
   - TAC parameters should return 2-byte values

### Method 2: Using ESPHome Service Discovery

The discovery configurations will attempt to log any advertised service UUIDs, but full service discovery requires connection and manual exploration.

### Common TAC Service Patterns

Based on the Navitas app analysis, look for:
- **Primary service** - Main communication service
- **Read characteristic** - For reading parameter values
- **Write characteristic** - For writing configuration parameters  
- **Notify characteristic** - For real-time updates (optional)

## Step 4: Testing and Validation

### Validate Your Discovery

Once you have potential UUIDs:

1. **Test basic connection:**
   ```yaml
   ble_client:
     - mac_address: "YOUR_DISCOVERED_MAC"
       id: test_client
   ```

2. **Test reading a characteristic:**
   ```yaml
   sensor:
     - platform: ble_client
       ble_client_id: test_client
       name: "Test Reading"
       service_uuid: "YOUR_SERVICE_UUID"
       characteristic_uuid: "YOUR_CHAR_UUID"
       lambda: |-
         uint16_t raw = x[0] | (x[1] << 8);
         return (float)raw;
   ```

3. **Look for reasonable values:**
   - Temperature: 20-80°C range
   - Voltage: 36-48V typical for golf carts
   - Current: 0-200A range
   - RPM: 0-4000 typical range

## Step 5: Integration with Main Config

### Update the Main Configuration

Once you've discovered working UUIDs:

1. **Update `secrets.yaml`:**
   ```yaml
   tac_controller_mac: "AA:BB:CC:DD:EE:FF"  # Your discovered MAC
   tac_service_uuid: "12345678-1234-1234-1234-123456789ABC"
   tac_read_characteristic: "87654321-4321-4321-4321-210987654321"
   ```

2. **Update `navitas_tac_controller.yaml`:**
   - Replace placeholder MAC address
   - Replace placeholder UUIDs with discovered ones
   - Test each sensor individually

## Troubleshooting Discovery

### No Devices Found

**Check:**
- Golf cart is powered on and TAC controller is active
- ESP32 is within BLE range (try moving closer)
- BLE scanning is enabled (`Auto Discovery` switch on)

**Try:**
- Restart the golf cart
- Power cycle the ESP32
- Use a phone BLE scanner app to verify devices are present

### Multiple Candidates Found

**Prioritize devices with:**
- Names containing "navitas", "tac", "golf"
- Strongest signal strength (highest RSSI)
- Multiple service UUIDs
- Highest confidence score

### Connection Fails

**Common issues:**
- Wrong MAC address format
- Device requires pairing (not typical for TAC controllers)
- Service UUIDs are incorrect
- Controller is in sleep mode

### No Data from Characteristics

**Check:**
- Characteristic UUIDs are correct
- Using the right service UUID
- Controller is actively running (golf cart on)
- Parameter addresses match TAC dictionary

## Advanced Discovery Techniques

### Using ESP32 BLE Scanner Sketch

If ESPHome discovery isn't working, try this Arduino sketch:

```cpp
#include "BLEDevice.h"
#include "BLEUtils.h"
#include "BLEScan.h"

BLEScan* pBLEScan;

void setup() {
  Serial.begin(115200);
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
}

void loop() {
  BLEScanResults foundDevices = pBLEScan->start(5, false);
  for (int i = 0; i < foundDevices.getCount(); i++) {
    BLEAdvertisedDevice device = foundDevices.getDevice(i);
    Serial.printf("Device: %s, RSSI: %d\n", 
                  device.getAddress().toString().c_str(), 
                  device.getRSSI());
    if (device.haveName()) {
      Serial.printf("Name: %s\n", device.getName().c_str());
    }
  }
  delay(10000);
}
```

### Monitoring BLE Traffic

Use tools like:
- **Wireshark** with BLE capture
- **Nordic nRF Sniffer** for detailed protocol analysis
- **Ubertooth One** for advanced BLE sniffing

## Expected Results

### Successful Discovery Output

```
[W][tac_discovery:123] *** POTENTIAL TAC CONTROLLER FOUND ***
[W][tac_discovery:124] Address: A1:B2:C3:D4:E5:F6
[W][tac_discovery:125] Name: Navitas-TAC-001
[W][tac_discovery:126] RSSI: -45 dB
[W][tac_discovery:127] Confidence: 90/100
[W][tac_discovery:128] Service UUID: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
[W][tac_discovery:129] Service UUID: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E
```

### Configuration Generation

```
[W][config_gen:145] Add to your main ESPHome config:
[W][config_gen:146] 
[W][config_gen:147] ble_client:
[W][config_gen:148]   - mac_address: "A1:B2:C3:D4:E5:F6"
[W][config_gen:149]     id: navitas_tac_ble_client
[W][config_gen:150]     auto_connect: true
```

## Security Considerations

- **Read-only access** is generally safe for monitoring
- **Avoid writing critical parameters** without understanding their function
- **Test thoroughly** before deploying in production
- **Golf cart safety systems** remain independent of this monitoring

## Next Steps

Once discovery is complete:
1. Update the main TAC controller configuration
2. Test all sensors and verify reasonable values
3. Set up Home Assistant dashboards
4. Configure alerts for fault conditions
5. Implement data logging for maintenance tracking

This discovery process should provide you with all the information needed to successfully connect to and monitor your Navitas TAC golf cart controller.