# Navitas TAC Protocol Documentation

This document describes the technical implementation details of the Navitas TAC BLE communication protocol.

## Protocol Overview

The Navitas TAC controller uses a custom binary protocol over Bluetooth Low Energy (BLE) for parameter monitoring and configuration. The protocol is based on reverse-engineering the official Navitas mobile application.

## BLE Communication

### Service and Characteristics

- **Service UUID**: `0xFFB0`
- **Write Characteristic**: `0xFFB1` - Used to send commands to controller
- **Notify Characteristic**: `0xFFB2` - Used to receive responses from controller

### Connection Process

1. **BLE Discovery**: Scan for devices advertising service `0xFFB0`
2. **Connection**: Connect to the target MAC address
3. **Service Discovery**: Locate the write and notify characteristics
4. **Enable Notifications**: Register for notifications on `0xFFB2`
5. **Protocol Handshake**: Initialize communication protocol

## TAC Binary Protocol

### Packet Structure

All TAC packets use the following format:

```
[STX][TYPE][SEQ][CMD][LEN][DATA...][CRC16][ETX]
```

**Header Fields:**
- `STX` (1 byte): Start of transmission (`0x02`)
- `TYPE` (3 bytes): Packet type (`TAC` or `TSX`)
- `SEQ` (1 byte): Sequence number (usually `0x00`)
- `CMD` (1 byte): Command code
- `LEN` (1 byte): Data payload length

**Trailer Fields:**
- `CRC16` (2 bytes): Fletcher-16 checksum
- `ETX` (1 byte): End of transmission (`0x03`)

### Command Types

The protocol supports different command types based on memory address ranges:

| Command | Value | Address Range | Description |
|---------|-------|---------------|-------------|
| `CMD_READ_PARAMS_LOW` | `0x20` | 0x00-0xFF | RAM parameters |
| `CMD_READ_PARAMS_MID` | `0x23` | 0x100-0x1FF | Extended parameters |
| `CMD_READ_PARAMS_HIGH` | `0x26` | 0x200-0x2FF | Advanced settings |
| `CMD_READ_PARAMS_FLASH` | `0x28` | 0x300+ | Flash memory config |

### Protocol Initialization

1. **Initial Handshake**: Send `0x41` to establish communication
2. **TSX Query**: Send TSX-type packet with command `0x28` and data `[0x01, 0x28]`
3. **Wait for Response**: Controller acknowledges and becomes ready
4. **Parameter Requests**: Send parameter read commands

### Parameter Reading

Parameters are requested in groups based on their address ranges. The controller responds with 16-bit values in big-endian format.

**Request Format:**
```
STX + "TAC" + 0x00 + CMD + LEN + [addresses...] + CRC + ETX
```

**Response Format:**
```
STX + "TAC" + SEQ + CMD + LEN + [value1_hi, value1_lo, value2_hi, value2_lo, ...] + CRC + ETX
```

## Parameter Mapping

### Core Parameters

Based on TACDictionary.xml from the official application:

| Address | Name | Scale | Unit | Description |
|---------|------|-------|------|-------------|
| 0x00 | PBTEMPC | ÷16 | °C | Controller temperature |
| 0x01 | VBATVDC | ÷128 | V | DC bus voltage |
| 0x02 | IBATADC | ÷16 | A | Battery current |
| 0x25 | MTTEMPC | ÷1 | °C | Motor temperature |
| 0x26 | ROTORRPM | ÷1 | RPM | Motor RPM (signed) |
| 0x56 | FilteredSOC_q12 | ÷40.96 | % | State of charge |
| 0xC8 | SWITCHBITS | ÷1 | - | Switch states |

### Vehicle Configuration Parameters

| Address | Name | Scale | Unit | Description |
|---------|------|-------|------|-------------|
| 0x9B | TIREDIAMETER | ÷64 | inches | Tire diameter |
| 0x9C | REARAXLERATIO | ÷128 | ratio | Rear axle ratio |
| 0x9D | MILESORKILOMETERS | ÷1 | - | Units (0=miles, 1=km) |

## Data Processing

### Signed Integer Handling

Some parameters (particularly RPM) are transmitted as signed 16-bit integers using two's complement representation:

```cpp
// Convert unsigned to signed for RPM
int16_t signed_value = (int16_t)raw_value;
float rpm = signed_value / scale_factor;
```

### Switch State Decoding

The SWITCHBITS parameter (0xC8) encodes gear position using bit patterns:

```cpp
uint16_t switch_bits = raw_value;

if ((switch_bits & 0x0006) == 0x0006) {
    // Both forward and reverse bits set = Neutral
    gear_state = "Neutral";
} else if ((switch_bits & 0x0002) == 0x0002) {
    // Forward bit only = Forward
    gear_state = "Forward";
} else if ((switch_bits & 0x0004) == 0x0004) {
    // Reverse bit only = Reverse
    gear_state = "Reverse";
} else {
    // Neither bit set = Neutral
    gear_state = "Neutral";
}
```

### Speed Calculation

Vehicle speed is calculated from motor RPM using the formula from NavitasMotorControllerModel.js:

```cpp
// Constants
const float inchesPerMinuteToMPH = 60.0f * 2.0f * M_PI * 1.57828e-5f;

// Calculate speed
float speed = inchesPerMinuteToMPH * (fabs(rpm) * tire_diameter / 2.0f) / rear_axle_ratio;

// Convert to km/h if needed
if (use_kilometers) {
    speed = speed * 1.609344f;
}
```

## Checksum Algorithm

The protocol uses Fletcher-16 checksum for data integrity:

```cpp
void fletcher16(std::vector<uint8_t> &packet) {
    uint32_t sum1 = 255;
    uint32_t sum2 = 255;

    // Calculate on all bytes except last 3 (CRC + ETX)
    int len = packet.size() - 3;
    int index = 0;

    while (len > 0) {
        int block_size = len > 21 ? 21 : len;
        len -= block_size;

        do {
            sum1 += packet[index];
            sum2 += sum1;
            ++index;
        } while (--block_size > 0);

        sum1 = (sum1 & 0xFF) + (sum1 >> 8);
        sum2 = (sum2 & 0xFF) + (sum2 >> 8);
    }

    // Final reduction and insertion
    uint32_t final_sum1 = (sum1 & 0xFF) + (sum1 >> 8);
    uint32_t final_sum2 = (sum2 & 0xFF) + (sum2 >> 8);

    packet[packet.size() - 3] = (uint8_t)final_sum1;
    packet[packet.size() - 2] = (uint8_t)final_sum2;
}
```

## BLE Transmission

### Chunked Data Transfer

BLE has a 20-byte MTU limit, so larger packets must be chunked:

```cpp
const size_t CHUNK_SIZE = 20;

while (bytes_sent < data.size()) {
    size_t chunk_size = std::min(CHUNK_SIZE, data.size() - bytes_sent);

    // Send chunk via BLE write
    esp_ble_gattc_write_char(..., data + bytes_sent, ...);

    bytes_sent += chunk_size;

    // 20ms delay between chunks
    if (bytes_sent < data.size()) {
        delay(20);
    }
}
```

### Response Assembly

Responses may arrive in multiple BLE notifications and must be reassembled:

1. **Parse Header**: Extract expected total length from first packet
2. **Buffer Continuation**: Append subsequent packets to buffer
3. **Validate Complete**: Check if buffer contains complete response
4. **Process Data**: Parse parameter values from complete packet

## Memory Types

The TAC controller organizes parameters into different memory regions:

### RAM Parameters (0x00-0xFF)
- Real-time operational data
- Updated continuously during operation
- Include temperatures, currents, voltages, RPM

### Extended Parameters (0x100-0x1FF)
- Additional monitoring data
- Less frequently updated
- May include advanced diagnostics

### High Parameters (0x200-0x2FF)
- System configuration and status
- Switch states and mode information
- Advanced controller settings

### Flash Parameters (0x300+)
- Persistent configuration stored in flash memory
- Vehicle-specific settings (tire size, gear ratios)
- Calibration and setup parameters

## Error Handling

### Common Issues

- **Connection timeouts**: Implement retry logic with backoff
- **Checksum failures**: Validate and discard corrupted packets
- **Missing responses**: Track pending requests and timeout
- **BLE disconnections**: Reinitialize protocol on reconnect

### Validation

- Parameter values should be within reasonable ranges
- Temperature readings typically 0-100°C
- Voltage readings should match battery specifications
- Current readings should be plausible for load conditions

## Implementation Notes

### Thread Safety
- BLE callbacks run in separate thread context
- Use proper synchronization for shared data
- ESPHome component callbacks are thread-safe

### Performance
- Group parameter requests to minimize BLE transactions
- Cache vehicle configuration parameters after first read
- Use appropriate update intervals (typically 2-5 seconds)

### Compatibility
- Different TAC firmware versions may have slight variations
- Some parameters may not be available on all controller models
- Always validate parameter availability before use

## References

This implementation is based on reverse-engineering the following sources:
- Official Navitas mobile application
- TACDictionary.xml parameter definitions
- NavitasMotorControllerModel.js calculation algorithms
- DeviceCommunication.cs protocol implementation





IBATLIMIT
SPEEDOMETERMAXSPEED
SOFTWAREREVISION
VTHROTTLEV
REVERSESW
FORWARDSW
CHARGERINTERLOCKIN

ChargeIdc_Adc_q4_limit

Complete Parameter Request/Response Analysis Using TACDictionary.xml

  Request Parameters (36 total):

  02 54 53 58 00 20 24 00 01 0F 10 16 19 1C 1D 20 25 2B 2F 58 5A 5B 5F 60 6A 6B 75 99 9A 9B 9C 9D 9F A0 A1 A2 A4 B3 B4 B6 BC C8

  Response Data (72 bytes = 36 parameters × 2 bytes each):

  02 6C 1C C9 0C 41 4B E4 00 00 00 00 00 00 00 00 2E 7D 00 22 16 D0 00 50 00 08 00 78 00 01 14 28 3C 28 00 00 00 01 00 8C 00 A6
  00 23 05 C0 06 38 00 00 00 00 00 00 00 00 00 00 00 00 10 00 30 00 7C

  Parsed Parameter Values (using big-endian):

  | Pos | Addr | Parameter                | Raw Value     | Scale | Calculated Value  | Unit                    |
  |-----|------|--------------------------|---------------|-------|-------------------|-------------------------|
  | 0   | 0    | PBTEMPC                  | 02 6C = 620   | ÷16   | 38.8°C            | Controller Temp         |
  | 1   | 1    | VBATVDC                  | 1C C9 = 7369  | ÷128  | 57.57V            | Battery Voltage         |
  | 2   | 15   | VTHROTTLEV               | 0C 41 = 3137  | ÷4096 | 0.77V             | Throttle Voltage        |
  | 3   | 16   | VBRAKEV                  | 4B E4 = 19428 | ÷4096 | 4.74V             | Brake Voltage           |
  | 4   | 22   | REVERSESW                | 00 00 = 0     | ÷1    | 0 (Off)           | Reverse Switch          |
  | 5   | 25   | FOOTSW                   | 00 00 = 0     | ÷1    | 0 (Off)           | Foot Switch             |
  | 6   | 28   | BRAKESW                  | 00 00 = 0     | ÷1    | 0 (Off)           | Brake Switch            |
  | 7   | 29   | FORWARDSW                | 00 00 = 0     | ÷1    | 0 (Off)           | Forward Switch          |
  | 8   | 32   | SOFTWAREREVISION         | 2E 7D = 11901 | ÷1000 | 11.901            | Firmware Rev            |
  | 9   | 37   | MTTEMPC                  | 00 22 = 34    | ÷1    | 34°C              | Motor Temperature       |
  | 10  | 43   | IBATLIMIT                | 16 D0 = 5840  | ÷16   | 365A              | Battery Discharge Limit |
  | 11  | 47   | OTWARNINGC               | 00 50 = 80    | ÷1    | 80°C              | Controller O/T Warning  |
  | 12  | 88   | STATE                    | 00 08 = 8     | ÷1    | 8                 | Controller State        |
  | 13  | 90   | NONFATALOTTRIPC          | 00 78 = 120   | ÷1    | 120°C             | Motor O/T Warning       |
  | 14  | 91   | KEYIN                    | 00 01 = 1     | ÷1    | 1 (On)            | Key In                  |
  | 15  | 95   | THROTTLEMIN              | 14 28 = 5160  | ÷4096 | 1.26V             | Throttle Min            |
  | 16  | 96   | THROTTLEMAX              | 3C 28 = 15400 | ÷4096 | 3.76V             | Throttle Max            |
  | 17  | 106  | TOWIN                    | 00 00 = 0     | ÷1    | 0                 | Tow Input               |
  | 18  | 107  | CHARGERINTERLOCKIN       | 00 01 = 1     | ÷1    | 1 (Not Connected) | Charger Interlock       |
  | 19  | 117  | MOTTRIPC                 | 00 8C = 140   | ÷1    | 140°C             | Motor O/T Trip          |
  | 20  | 153  | Options                  | 00 A6 = 166   | ÷1    | 166               | Manufacturing Config    |
  | 21  | 154  | SPEEDOMETERMAXSPEED      | 00 23 = 35    | ÷1    | 35                | Speedometer Max Speed   |
  | 22  | 155  | TIREDIAMETER             | 05 C0 = 1472  | ÷64   | 23.0 inches ✓     | Tire Diameter           |
  | 23  | 156  | REARAXLERATIO            | 06 38 = 1592  | ÷128  | 12.44 ✓           | Rear Axle Ratio         |
  | 24  | 157  | MILESORKILOMETERS        | 00 00 = 0     | ÷1    | 0 (Miles) ✓       | Units                   |
  | 25  | 159  | GROUPONEFAULTS           | 00 00 = 0     | ÷1    | 0                 | Group One Faults        |
  | 26  | 160  | GROUPTWOFAULTS           | 00 00 = 0     | ÷1    | 0                 | Group Two Faults        |
  | 27  | 161  | GROUPTHREEFAULTS         | 00 00 = 0     | ÷1    | 0                 | Group Three Faults      |
  | 28  | 162  | GROUPFOURFAULTS          | 00 00 = 0     | ÷1    | 0                 | Group Four Faults       |
  | 29  | 164  | VEHICLELOCKED            | 00 00 = 0     | ÷1    | 0 (Unlocked)      | Vehicle Locked          |
  | 30  | 179  | BRAKEMIN                 | 00 00 = 0     | ÷4096 | 0.00V             | Brake Min               |
  | 31  | 180  | BRAKEMAX                 | 00 00 = 0     | ÷4096 | 0.00V             | Brake Max               |
  | 32  | 182  | THROTTLEFULL             | 10 00 = 4096  | ÷4096 | 1.00V             | Throttle Full           |
  | 33  | 188  | LOGICSIDEBATTERYVOLTAGEV | 30 00 = 12288 | ÷128  | 96.0V             | Logic Side Battery      |
  | 34  | 200  | SWITCHBITS               | 7C = 124      | ÷1    | 124               | Switch Bits ✓           |
