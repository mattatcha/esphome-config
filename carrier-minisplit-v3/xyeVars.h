#include "esphome.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>
#include <vector>


// NOTE: upstream xyeVars.h ships with RX=16 / TX=17, but the project README
// (and v1/v2 of this device) wire ESP TX→GPIO16 and ESP RX→GPIO17. This
// header matches the README/v1/v2 wiring; the upstream defines were the bug.
#define RX_PIN 17
#define TX_PIN 16

#define CMD_VAL 1
#define SEND_MODE 6
#define SEND_FAN 7
#define SEND_TEMP 8
#define SEND_CRC 14
#define SEND_LEN 16

#define REC_MODE 8
#define REC_FAN 9
#define REC_TEMP 10
#define T1_INDEX 11
#define T2A_INDEX 12
#define T2B_INDEX 13
#define T3_INDEX 14

using namespace esphome;

#if defined(ESP8266)
    #define IS_8266 1
    HardwareSerial mySerial(0);
#else
    #define IS_8266 0
    HardwareSerial mySerial(2);
#endif

class xyeVars {
  public:
    uint8_t setTemp = 70;
    uint8_t sendTimeCount = 0;
    bool newInput = false;
    std::string op = "off";
    std::string fan = "auto";
    uint8_t fanBytes = 0x00;
    uint8_t opBytes = 0x00;
    bool doneReading = false;
    bool hasValidState = false;
    uint8_t waitCount = 0;
    uint8_t prevResp = 0;
    bool waitingToSend = false;
    bool waitingForResponse = false;
    bool commandSent = false;
    uint8_t recData[30] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t sendData[16] = {0xaa, 0xc3, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x00, 0x55};
    uint8_t waitSendData[16] = {0xaa, 0xc3, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x00, 0x55};
    uint8_t queryData[16] = {0xaa, 0xc0, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x81, 0x55};
    uint8_t checkData[6] = {0xaa, 0xc0, 0x80, 0x00, 0x00, 0x00};
} xyeVars;
