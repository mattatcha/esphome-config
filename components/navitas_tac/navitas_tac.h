#pragma once

#include "esphome/core/component.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#include <vector>
#include <map>

namespace esphome {
namespace navitas_tac {

// TAC Protocol constants
static const uint8_t TAC_STX = 0x02;
static const uint8_t TAC_ETX = 0x03;
static const std::vector<uint8_t> TAC_TYPE = {'T', 'A', 'C'};
static const std::vector<uint8_t> TSX_TYPE = {'T', 'S', 'X'};
static const uint8_t CMD_READ_PARAMS_LOW = 0x20;    // Address < 256
static const uint8_t CMD_READ_PARAMS_MID = 0x23;    // Address >= 256 && < 512
static const uint8_t CMD_READ_PARAMS_HIGH = 0x26;   // Address >= 512 && < 768
static const uint8_t CMD_READ_PARAMS_FLASH = 0x28;  // Address >= 768 (Flash memory)

// BLE Service and Characteristic UUIDs
static const uint16_t SERVICE_UUID = 0xFFB0;
static const uint16_t WRITE_CHARACTERISTIC_UUID = 0xFFB1;
static const uint16_t NOTIFY_CHARACTERISTIC_UUID = 0xFFB2;

struct Parameter {
  uint8_t address;
  std::string name;
  float scale_factor;
  std::string unit;
};

class NavitasTAC : public PollingComponent, public esphome::ble_client::BLEClientNode {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_temperature_sensor(sensor::Sensor *sensor) { temperature_sensor_ = sensor; }
  void set_motor_temperature_sensor(sensor::Sensor *sensor) { motor_temperature_sensor_ = sensor; }
  void set_voltage_sensor(sensor::Sensor *sensor) { voltage_sensor_ = sensor; }
  void set_current_sensor(sensor::Sensor *sensor) { current_sensor_ = sensor; }
  void set_motor_rpm_sensor(sensor::Sensor *sensor) { motor_rpm_sensor_ = sensor; }
  void set_speed_sensor(sensor::Sensor *sensor) { speed_sensor_ = sensor; }
  void set_soc_sensor(sensor::Sensor *sensor) { soc_sensor_ = sensor; }
  void set_state_sensor(text_sensor::TextSensor *sensor) { state_sensor_ = sensor; }
  void set_connected_sensor(binary_sensor::BinarySensor *sensor) { connected_sensor_ = sensor; }

  // Vehicle configuration for speed calculation
  void set_tire_diameter(float diameter) { tire_diameter_ = diameter; }
  void set_rear_axle_ratio(float ratio) { rear_axle_ratio_ = ratio; }
  void set_use_kilometers(bool use_km) { use_kilometers_ = use_km; }

  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override;

 protected:
  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *motor_temperature_sensor_{nullptr};
  sensor::Sensor *voltage_sensor_{nullptr};
  sensor::Sensor *current_sensor_{nullptr};
  sensor::Sensor *motor_rpm_sensor_{nullptr};
  sensor::Sensor *speed_sensor_{nullptr};
  sensor::Sensor *soc_sensor_{nullptr};
  text_sensor::TextSensor *state_sensor_{nullptr};
  binary_sensor::BinarySensor *connected_sensor_{nullptr};

  // Protocol state
  std::vector<uint8_t> response_buffer_;
  uint32_t expected_response_length_{0};
  bool protocol_initialized_{false};
  uint8_t sequence_number_{0};

  // Parameter request tracking
  std::vector<uint8_t> requested_parameters_;
  bool parameters_built_{false};

  // BLE handles
  uint16_t write_handle_{0};
  uint16_t notify_handle_{0};

  // Vehicle configuration for speed calculation
  float tire_diameter_{22.0f};    // Default 22 inches (golf cart typical)
  float rear_axle_ratio_{12.5f};  // Default gear ratio
  bool use_kilometers_{false};    // Default to mph
  float current_rpm_{0.0f};       // Store current RPM for speed calculation

  // Vehicle configuration from controller (overrides defaults when received)
  float controller_tire_diameter_{0.0f};    // From TIREDIAMETER parameter
  float controller_rear_axle_ratio_{0.0f};  // From REARAXLERATIO parameter
  bool controller_use_kilometers_{false};   // From MILESORKILOMETERS parameter
  bool vehicle_config_received_{false};     // Track if config has been loaded from controller

  // Methods
  void init_protocol_();
  void send_tsx_query_();
  void build_parameter_request_();
  void send_parameter_request_();
  void process_notification_(const std::vector<uint8_t> &data);
  void process_complete_response_(const std::vector<uint8_t> &buffer);
  void parse_parameters_(const std::vector<uint8_t> &buffer, uint8_t data_length);
  std::vector<uint8_t> create_packet_(const std::vector<uint8_t> &type, uint8_t command,
                                      const std::vector<uint8_t> &data);
  void fletcher16_(std::vector<uint8_t> &packet);
  void update_connection_status_(bool connected);
  void write_chunked_data_(const std::vector<uint8_t> &data);
  void calculate_and_publish_speed_();
  uint8_t get_command_for_address_(uint16_t address);

  // Parameter mapping from TACDictionary.xml
  static const std::map<uint8_t, Parameter> parameter_map_;
};

}  // namespace navitas_tac
}  // namespace esphome
