# Navitas TAC Controller BLE UUIDs

This document contains the Bluetooth Low Energy (BLE) service and characteristic UUIDs discovered from decompiling the official Navitas Beta Android application.

## Service UUIDs

The Navitas TAC controller advertises two primary services:

### Primary Service
- **UUID**: `0000fff0-0000-1000-8000-00805f9b34fb`
- **Usage**: Main communication service for TAC controllers

### Secondary Service  
- **UUID**: `0000ffb0-0000-1000-8000-00805f9b34fb`
- **Usage**: Alternative service (possibly for different controller types or firmware versions)

## Characteristic UUIDs

Each service contains two characteristics for bi-directional communication:

### Write Characteristics (Client → Controller)
- **UUID 1**: `0000fff1-0000-1000-8000-00805f9b34fb` (for fff0 service)
- **UUID 2**: `0000ffb1-0000-1000-8000-00805f9b34fb` (for ffb0 service)
- **Properties**: Write without response
- **Usage**: Send commands and data requests to the controller

### Read/Notify Characteristics (Controller → Client)
- **UUID 1**: `0000fff2-0000-1000-8000-00805f9b34fb` (for fff0 service)
- **UUID 2**: `0000ffb2-0000-1000-8000-00805f9b34fb` (for ffb0 service)
- **Properties**: Read, Notify
- **Usage**: Receive data responses and notifications from the controller

## Protocol Details

### Packet Structure
Based on analysis of the `DeviceComunication.cs` file, the TAC protocol uses the following packet structure:

```
[STX][TYPE1][TYPE2][TYPE3][SEQ][CMD][LEN][DATA...][CRC1][CRC2][ETX]
```

- **STX** (1 byte): Start of transmission = `0x02`
- **TYPE** (3 bytes): Controller type = `"TAC"` (`0x54 0x41 0x43`)
- **SEQ** (1 byte): Sequence number
- **CMD** (1 byte): Command code
- **LEN** (1 byte): Data length
- **DATA** (variable): Parameter data
- **CRC** (2 bytes): Fletcher-16 checksum
- **ETX** (1 byte): End of transmission = `0x03`

### Command Codes
From the analysis, common command codes include:
- `0x20` (32): Read parameters from address range 0x00-0xFF
- `0x23` (35): Read parameters from address range 0x100-0x1FF
- `0x26` (38): Read parameters from address range 0x200-0x2FF
- `0x28` (40): Read parameters from address range 0x300-0x3FF

### Checksum Algorithm
The protocol uses Fletcher-16 checksum algorithm:

```cpp
void fletcher16(uint8_t* checksum, uint8_t* data, int length) {
    uint8_t sum1 = 255, sum2 = 255;
    for (int i = 0; i < length - 3; i++) {
        sum1 += data[i];
        sum2 += sum1;
        sum1 = (sum1 & 0xFF) + (sum1 >> 8);
        sum2 = (sum2 & 0xFF) + (sum2 >> 8);
    }
    sum1 = (sum1 & 0xFF) + (sum1 >> 8);
    sum2 = (sum2 & 0xFF) + (sum2 >> 8);
    checksum[0] = sum1;
    checksum[1] = sum2;
}
```

## Controller States

The controller reports various operational states:

| State | Value | Description |
|-------|-------|-------------|
| Reset | 0 | Controller reset state |
| Charger Check | 1 | Charger or tow connection check |
| Throttle Zero Check | 2 | Throttle zero startup check |
| Throttle Saturation Check | 3 | Throttle saturation startup check |
| Reverse Buzzer Check | 4 | Reverse buzzer check |
| DC Bus Check | 5 | DC bus check |
| Parking Brake Check | 6 | Startup parking brake check |
| Startup | 7 | Startup sequence |
| Neutral | 8 | Neutral state |
| Neutral Transition | 9 | Transitioning to neutral |
| Forward | 10 | Forward motion |
| Reverse | 11 | Reverse motion |
| DropBrake | 12 | Drop brake state |
| Error | 13 | Error state |

## Special Parameters

Some parameters have special handling:
- **Parameter 49**: Save configuration (TAC)
- **Parameter 50**: Initialize configuration (TAC)
- **Parameter 34**: Get scope block data (TAC)
- **Parameter 37**: Get flash block data (TAC)
- **Parameter 109**: Enter bootloader mode (TAC)
- **Parameter 199**: Save configuration (TSX)
- **Parameter 255**: Special commands (TSX) - "Bootload", "FastDatalog"

## Usage Notes

1. The controller type is determined by the response to initial communication attempts
2. Different firmware versions may support different address ranges
3. The app checks software revision to determine available features:
   - Address range 0x100-0x1FF requires software revision > 1.28
   - Address range 0x200-0x2FF requires software revision > 5.0
   - Address range 0x300-0x3FF requires software revision > 15.4
4. Communication timeout is typically 3-4 seconds for normal operations, up to 120 seconds for bootloader operations

## Files Generated

1. **navitas_tac_discovery.yaml**: ESPHome configuration for discovering TAC controllers
2. **navitas_tac_client.yaml**: ESPHome configuration for connecting to and communicating with TAC controllers
3. **tac_client.h**: Custom C++ component for handling TAC protocol communication

## Setup Instructions

1. Use `navitas_tac_discovery.yaml` to scan for and identify TAC controllers in your area
2. Note the MAC address of your TAC controller from the discovery logs
3. Update the `mac_address` field in `navitas_tac_client.yaml` with your controller's MAC address
4. Flash the client configuration to your ESP32
5. Monitor the logs to see parameter data being received from the controller

The scaling factors and parameter mappings may need adjustment based on your specific controller model and firmware version.