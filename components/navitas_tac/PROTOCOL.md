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