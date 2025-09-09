#include "navitas_tac.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/components/esp32_ble/ble_uuid.h"

namespace esphome {
namespace navitas_tac {

static const char *TAG = "navitas_tac";

// Parameter mapping from TACDictionary.xml - extended mapping
const std::map<uint8_t, Parameter> NavitasTAC::parameter_map_ = {
    // Core parameters
    {0x00, {0x00, "Controller Temperature", 16.0f, "°C"}},  // PBTEMPC
    {0x01, {0x01, "DC Bus Voltage", 128.0f, "V"}},          // VBATVDC
    {0x02, {0x02, "Battery Current", 16.0f, "A"}},          // IBATADC
    {0x25, {0x25, "Motor Temperature", 1.0f, "°C"}},        // MTTEMPC - address 37

    // Motor RPM for speed calculation (ROTORRPM at address 38 decimal = 0x26 hex)
    {0x26, {0x26, "Motor RPM", 1.0f, "RPM"}},  // ROTORRPM

    // State of Charge (FilteredSOC_q12 at address 86 decimal = 0x56 hex)
    {0x56, {0x56, "State of Charge", 40.96f, "%"}},  // FilteredSOC_q12 - address 86

    // Switch states (SWITCHBITS at address 200 decimal = 0xC8 hex)
    {0xC8, {0xC8, "Switch Bits", 1.0f, ""}},  // SWITCHBITS - address 200

    // Vehicle configuration parameters for speed calculation
    {0x9B, {0x9B, "Tire Diameter", 64.0f, "inches"}},    // TIREDIAMETER - address 155
    {0x9C, {0x9C, "Rear Axle Ratio", 128.0f, "ratio"}},  // REARAXLERATIO - address 156
    {0x9D, {0x9D, "Miles or Kilometers", 1.0f, ""}},     // MILESORKILOMETERS - address 157

};

void NavitasTAC::setup() { ESP_LOGCONFIG(TAG, "Setting up Navitas TAC Controller..."); }

void NavitasTAC::dump_config() {
  ESP_LOGCONFIG(TAG, "Navitas TAC Controller:");
  LOG_SENSOR("  ", "Controller Temperature", this->temperature_sensor_);
  LOG_SENSOR("  ", "Motor Temperature", this->motor_temperature_sensor_);
  LOG_SENSOR("  ", "DC Bus Voltage", this->voltage_sensor_);
  LOG_SENSOR("  ", "Battery Current", this->current_sensor_);
  LOG_SENSOR("  ", "Motor RPM", this->motor_rpm_sensor_);
  LOG_SENSOR("  ", "Speed", this->speed_sensor_);
  LOG_SENSOR("  ", "SOC", this->soc_sensor_);
  LOG_TEXT_SENSOR("  ", "State", this->state_sensor_);
  LOG_TEXT_SENSOR("  ", "Gear", this->gear_sensor_);
  LOG_BINARY_SENSOR("  ", "Connected", this->connected_sensor_);

  if (this->speed_sensor_ != nullptr) {
    ESP_LOGCONFIG(TAG,
                  "  Vehicle Configuration:\n"
                  "    Tire Diameter: %.1f inches\n"
                  "    Rear Axle Ratio: %.2f\n"
                  "    Units: %s",
                  this->tire_diameter_, this->rear_axle_ratio_, this->use_kilometers_ ? "kilometers" : "miles");
  }
}

void NavitasTAC::update() {
  if (this->node_state != esp32_ble_tracker::ClientState::ESTABLISHED) {
    ESP_LOGW(TAG, "Not connected to TAC controller, skipping update");
    return;
  }

  // Build parameter request once after protocol initialization
  if (this->protocol_initialized_ && !this->parameters_built_) {
    ESP_LOGI(TAG, "Building parameter request for the first time");
    this->build_parameter_request_();
    this->parameters_built_ = true;
  }

  // Send parameter request if we have parameters and protocol is ready
  if (this->protocol_initialized_ && this->parameters_built_) {
    this->send_parameter_request_();
  }
}

void NavitasTAC::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                     esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_OPEN_EVT:
      if (param->open.status == ESP_GATT_OK) {
        ESP_LOGI(TAG, "Connected to TAC Controller");
        this->update_connection_status_(true);
      }
      break;

    case ESP_GATTC_DISCONNECT_EVT:
      ESP_LOGI(TAG, "Disconnected from TAC Controller");
      this->node_state = esp32_ble_tracker::ClientState::IDLE;
      this->update_connection_status_(false);
      this->protocol_initialized_ = false;
      break;

    case ESP_GATTC_SEARCH_CMPL_EVT: {
      // Find write and notify characteristics
      auto *write_char = this->parent_->get_characteristic(SERVICE_UUID, WRITE_CHARACTERISTIC_UUID);
      auto *notify_char = this->parent_->get_characteristic(SERVICE_UUID, NOTIFY_CHARACTERISTIC_UUID);

      if (write_char != nullptr && notify_char != nullptr) {
        this->write_handle_ = write_char->handle;
        this->notify_handle_ = notify_char->handle;

        ESP_LOGI(TAG,
                 "BLE Characteristics Found:\n"
                 "  Write Handle: 0x%04x\n"
                 "  Notify Handle: 0x%04x",
                 this->write_handle_, this->notify_handle_);

        // Enable notifications
        auto status = esp_ble_gattc_register_for_notify(gattc_if, this->parent_->get_remote_bda(), notify_char->handle);
        if (status) {
          ESP_LOGW(TAG, "Failed to enable notifications, status=%d", status);
        } else {
          // Start protocol initialization
          this->set_timeout("init_protocol", 1000, [this]() { this->init_protocol_(); });
        }
      } else {
        ESP_LOGE(TAG, "Required BLE characteristics not found");
      }
      break;
    }
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
      this->node_state = esp32_ble_tracker::ClientState::ESTABLISHED;
      break;
    }

    case ESP_GATTC_NOTIFY_EVT:
      if (param->notify.handle == this->notify_handle_) {
        std::vector<uint8_t> data(param->notify.value, param->notify.value + param->notify.value_len);
        this->process_notification_(data);
      }
      break;

    default:
      break;
  }
}

void NavitasTAC::init_protocol_() {
  ESP_LOGD(TAG, "Initializing TAC protocol handshake");

  // Send initial handshake (0x41)
  std::vector<uint8_t> handshake = {0x41};
  this->write_chunked_data_(handshake);

  // Send TSX query after delay
  this->set_timeout("tsx_query", 500, [this]() { this->send_tsx_query_(); });
}

void NavitasTAC::send_tsx_query_() {
  ESP_LOGD(TAG, "Querying controller type");

  std::vector<uint8_t> data = {0x01, 0x28};
  auto packet = this->create_packet_(TSX_TYPE, CMD_READ_PARAMS_LOW, data);

  this->write_chunked_data_(packet);
}

void NavitasTAC::build_parameter_request_() {
  ESP_LOGI(TAG, "Building parameter request based on configured sensors");

  this->requested_parameters_.clear();

  // Add parameters based on configured sensors
  if (this->temperature_sensor_ != nullptr) {
    this->requested_parameters_.push_back(0x00);  // PBTEMPC
    ESP_LOGD(TAG, "Added controller temperature parameter (0x00)");
  }

  if (this->motor_temperature_sensor_ != nullptr) {
    this->requested_parameters_.push_back(0x25);  // MTTEMPC
    ESP_LOGD(TAG, "Added motor temperature parameter (0x25)");
  }

  if (this->voltage_sensor_ != nullptr) {
    this->requested_parameters_.push_back(0x01);  // VBATVDC
    ESP_LOGD(TAG, "Added voltage parameter (0x01)");
  }

  if (this->current_sensor_ != nullptr) {
    this->requested_parameters_.push_back(0x02);  // IBATADC
    ESP_LOGD(TAG, "Added current parameter (0x02)");
  }

  if (this->motor_rpm_sensor_ != nullptr || this->speed_sensor_ != nullptr) {
    this->requested_parameters_.push_back(0x26);  // ROTORRPM
    ESP_LOGD(TAG, "Added motor RPM parameter (0x26)");

    // Request vehicle configuration parameters for accurate speed calculation if speed sensor is configured
    if (this->speed_sensor_ != nullptr && !this->vehicle_config_received_) {
      this->requested_parameters_.push_back(0x9B);  // TIREDIAMETER (155)
      this->requested_parameters_.push_back(0x9C);  // REARAXLERATIO (156)
      this->requested_parameters_.push_back(0x9D);  // MILESORKILOMETERS (157)
      ESP_LOGD(TAG, "Added vehicle config parameters for speed calculation");
    }
  }

  if (this->soc_sensor_ != nullptr) {
    this->requested_parameters_.push_back(0x56);  // FilteredSOC_q12
    ESP_LOGD(TAG, "Added SOC parameter (0x56)");
  }

  if (this->gear_sensor_ != nullptr) {
    this->requested_parameters_.push_back(0xC8);  // SWITCHBITS
    ESP_LOGD(TAG, "Added gear state parameter (0xC8)");
  }

  if (this->requested_parameters_.empty()) {
    ESP_LOGW(TAG, "No parameters configured for monitoring");
    return;
  }

  // Log the parameter list
  std::string param_list = "";
  for (size_t i = 0; i < this->requested_parameters_.size(); i++) {
    param_list += str_sprintf("0x%02X", this->requested_parameters_[i]);
    if (i < this->requested_parameters_.size() - 1)
      param_list += ", ";
  }
  ESP_LOGI(TAG, "Monitoring %d parameters: [%s]", this->requested_parameters_.size(), param_list.c_str());
}

void NavitasTAC::send_parameter_request_() {
  if (this->requested_parameters_.empty()) {
    ESP_LOGW(TAG, "No parameters to request");
    return;
  }

  // Group parameters by command type to send separate requests
  std::map<uint8_t, std::vector<uint8_t>> grouped_params;

  for (uint8_t param_addr : this->requested_parameters_) {
    uint16_t address = param_addr;  // Convert to 16-bit for address check
    uint8_t command = this->get_command_for_address_(address);
    grouped_params[command].push_back(param_addr);
  }

  // Send separate request for each command type
  for (const auto &group : grouped_params) {
    uint8_t command = group.first;
    const std::vector<uint8_t> &params = group.second;

    auto packet = this->create_packet_(TSX_TYPE, command, params);
    ESP_LOGV(TAG, "Requesting %d parameters (cmd 0x%02X): %s", params.size(), command,
             format_hex_pretty(packet.data(), packet.size()).c_str());

    this->write_chunked_data_(packet);
    delay(50);  // Small delay between different command requests
  }
}

void NavitasTAC::process_notification_(const std::vector<uint8_t> &data) {
  ESP_LOGVV(TAG, "Received packet %s", format_hex_pretty(data.data(), data.size()).c_str());

  // Check if this is the start of a new TAC response
  if (data.size() >= 6 && data[0] == TAC_STX && data[1] == 'T' && data[2] == 'A' && data[3] == 'C') {
    uint8_t sequence = data[4];
    uint8_t command = data[5];
    uint8_t data_length = data[6];

    ESP_LOGV(TAG, "SEQ: 0x%02X, CMD: 0x%02X, LEN: %d", sequence, command, data_length);

    // Initialize response buffer with expected total length (data_length + 10 bytes)
    this->expected_response_length_ = data_length + 10;
    this->response_buffer_.clear();

    // Add current packet data to buffer
    this->response_buffer_.insert(this->response_buffer_.end(), data.begin(), data.end());

    if (this->response_buffer_.size() != this->expected_response_length_) {
      ESP_LOGD(TAG, "Response size mismatch: got %d, expected %d", this->response_buffer_.size(),
               this->expected_response_length_);
    }

  } else {
    // This is a continuation packet - add to buffer
    if (this->expected_response_length_ > 0) {
      this->response_buffer_.insert(this->response_buffer_.end(), data.begin(), data.end());
      ESP_LOGVV(TAG, "Added continuation data, buffer now has %d bytes", this->response_buffer_.size());
    }
  }

  // Check if we have complete response
  if (this->expected_response_length_ > 0 && this->response_buffer_.size() >= this->expected_response_length_) {
    this->process_complete_response_(this->response_buffer_);

    // Reset for next response
    this->expected_response_length_ = 0;
    this->response_buffer_.clear();
    this->protocol_initialized_ = true;
  }
}

void NavitasTAC::process_complete_response_(const std::vector<uint8_t> &buffer) {
  if (buffer.size() < 7)
    return;

  uint8_t command = buffer[5];
  uint8_t data_length = buffer[6];

  // Parse parameter data for command 0x20 (read parameters)
  if (command == CMD_READ_PARAMS_LOW) {
    this->parse_parameters_(buffer, data_length);
  }
}

void NavitasTAC::parse_parameters_(const std::vector<uint8_t> &buffer, uint8_t data_length) {
  // Parameters are 2-byte values, start at position 7
  for (int i = 7; i < 7 + data_length; i += 2) {
    if (i + 1 < buffer.size()) {
      uint16_t value = (buffer[i] << 8) | buffer[i + 1];  // Big endian
      int param_index = (i - 7) / 2;

      // Map parameter index to actual address from our request
      uint8_t param_addr = 0xFF;
      if (param_index < this->requested_parameters_.size()) {
        param_addr = this->requested_parameters_[param_index];
      }

      ESP_LOGV(TAG, "Param[%d] addr 0x%02X: %d (0x%04X)", param_index, param_addr, value, value);

      // Decode based on parameter mapping
      auto it = parameter_map_.find(param_addr);
      if (it != parameter_map_.end()) {
        const auto &param = it->second;
        float scaled_value = value / param.scale_factor;

        ESP_LOGD(TAG, "%s (0x%02X): %.2f %s", param.name.c_str(), param_addr, scaled_value, param.unit.c_str());

        // Update appropriate sensors based on parameter address
        if (param_addr == 0x00 && this->temperature_sensor_ != nullptr && scaled_value < 200) {
          this->temperature_sensor_->publish_state(scaled_value);
        } else if (param_addr == 0x25 && this->motor_temperature_sensor_ != nullptr && scaled_value < 200) {
          this->motor_temperature_sensor_->publish_state(scaled_value);
        } else if (param_addr == 0x01 && this->voltage_sensor_ != nullptr && scaled_value > 0 && scaled_value < 100) {
          this->voltage_sensor_->publish_state(scaled_value);
        } else if (param_addr == 0x02 && this->current_sensor_ != nullptr && scaled_value < 500) {
          this->current_sensor_->publish_state(scaled_value);
        } else if (param_addr == 0x26) {
          // ROTORRPM received - handle as signed integer for reverse detection
          // Reinterpret the 16-bit value as signed (two's complement) to handle negative RPM
          int16_t signed_value = (int16_t)value;  // Reinterpret bits as signed
          float signed_rpm = signed_value / param.scale_factor;
          
          this->current_rpm_ = signed_rpm;  // Store signed RPM value
          if (this->motor_rpm_sensor_ != nullptr) {
            this->motor_rpm_sensor_->publish_state(signed_rpm);
          }
          if (this->speed_sensor_ != nullptr) {
            this->calculate_and_publish_speed_();
          }
        } else if (param_addr == 0x56 && this->soc_sensor_ != nullptr && scaled_value >= 0 && scaled_value <= 100) {
          // FilteredSOC_q12 received - publish state of charge
          this->soc_sensor_->publish_state(scaled_value);
        } else if (param_addr == 0xC8 && this->gear_sensor_ != nullptr) {
          // SWITCHBITS received - parse gear state (following Navitas JavaScript logic)
          uint16_t switch_bits = (uint16_t)scaled_value;
          
          std::string gear_state;
          if ((switch_bits & 0x0006) == 0x0006) {
            // Both direction switches on - Neutral
            gear_state = "Neutral";
          } else if ((switch_bits & 0x0002) == 0x0002) {
            // Forward switch only - Forward
            gear_state = "Forward";
          } else if ((switch_bits & 0x0004) == 0x0004) {
            // Reverse switch only - Reverse  
            gear_state = "Reverse";
          } else {
            // Neither switch on - Neutral
            gear_state = "Neutral";
          }
          
          ESP_LOGD(TAG, "Gear state: %s (switch_bits=0x%04X)", 
                   gear_state.c_str(), switch_bits);
          this->gear_sensor_->publish_state(gear_state);
        } else if (param_addr == 0x9B) {
          // TIREDIAMETER received - store controller value
          this->controller_tire_diameter_ = scaled_value;
          ESP_LOGD(TAG, "Vehicle config - tire diameter: %.1f inches", scaled_value);
        } else if (param_addr == 0x9C) {
          // REARAXLERATIO received - store controller value
          this->controller_rear_axle_ratio_ = scaled_value;
          ESP_LOGD(TAG, "Vehicle config - rear axle ratio: %.2f", scaled_value);
        } else if (param_addr == 0x9D) {
          // MILESORKILOMETERS received - store controller value
          this->controller_use_kilometers_ = (scaled_value != 0);
          this->vehicle_config_received_ = true;  // Mark as received

          ESP_LOGI(TAG,
                   "Vehicle Configuration Loaded:\n"
                   "  Tire Diameter: %.1f inches\n"
                   "  Rear Axle Ratio: %.2f\n"
                   "  Units: %s",
                   this->controller_tire_diameter_, this->controller_rear_axle_ratio_,
                   this->controller_use_kilometers_ ? "kilometers" : "miles");

          // Rebuild parameter request to exclude vehicle config parameters on future requests
          this->build_parameter_request_();
        } else if (param_addr == 0x30 && this->soc_sensor_ != nullptr && scaled_value >= 0 && scaled_value <= 100) {
          this->soc_sensor_->publish_state(scaled_value);
        }
      } else {
        // Log unknown parameters with various scale factors for analysis
        ESP_LOGD(TAG, "Unknown parameter 0x%02X: raw=%d (%.1f@÷16, %.2f@÷128, %.3f@÷256)", param_addr, value,
                 value / 16.0f, value / 128.0f, value / 256.0f);
      }
    }
  }
}

std::vector<uint8_t> NavitasTAC::create_packet_(const std::vector<uint8_t> &type, uint8_t command,
                                                const std::vector<uint8_t> &data) {
  std::vector<uint8_t> packet;

  packet.push_back(TAC_STX);                              // STX
  packet.insert(packet.end(), type.begin(), type.end());  // Type (TAC/TSX)
  packet.push_back(0x00);                                 // Sequence
  packet.push_back(command);                              // Command
  packet.push_back(data.size());                          // Length
  packet.insert(packet.end(), data.begin(), data.end());  // Data

  // Add placeholders for checksum and ETX
  packet.push_back(0x00);     // Checksum[0]
  packet.push_back(0x00);     // Checksum[1]
  packet.push_back(TAC_ETX);  // ETX

  // Calculate and insert Fletcher-16 checksum
  this->fletcher16_(packet);

  return packet;
}

void NavitasTAC::fletcher16_(std::vector<uint8_t> &packet) {
  uint32_t sum1 = 255;
  uint32_t sum2 = 255;

  // Calculate checksum on all bytes except last 3 (2-byte checksum + ETX)
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

  // Final reduction
  uint32_t final_sum1 = (sum1 & 0xFF) + (sum1 >> 8);
  uint32_t final_sum2 = (sum2 & 0xFF) + (sum2 >> 8);

  // Insert checksum at positions -3 and -2
  packet[packet.size() - 3] = (uint8_t) final_sum1;
  packet[packet.size() - 2] = (uint8_t) final_sum2;
}

void NavitasTAC::update_connection_status_(bool connected) {
  if (this->connected_sensor_ != nullptr) {
    this->connected_sensor_->publish_state(connected);
  }

  if (this->state_sensor_ != nullptr) {
    this->state_sensor_->publish_state(connected ? "Connected" : "Disconnected");
  }
}

void NavitasTAC::write_chunked_data_(const std::vector<uint8_t> &data) {
  const size_t CHUNK_SIZE = 20;

  if (data.empty()) {
    ESP_LOGW(TAG, "Attempted to write empty data");
    return;
  }

  size_t bytes_sent = 0;
  while (bytes_sent < data.size()) {
    size_t chunk_size = std::min(CHUNK_SIZE, data.size() - bytes_sent);

    auto status = esp_ble_gattc_write_char(
        this->parent_->get_gattc_if(), this->parent_->get_conn_id(), this->write_handle_, chunk_size,
        const_cast<uint8_t *>(data.data() + bytes_sent), ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);

    if (status != ESP_OK) {
      ESP_LOGW(TAG, "Failed to write chunk at offset %d, status=%d", bytes_sent, status);
      return;
    }

    ESP_LOGVV(TAG, "Sent chunk: %d bytes at offset %d", chunk_size, bytes_sent);
    bytes_sent += chunk_size;

    // Add 20ms delay between chunks (matching C# implementation)
    if (bytes_sent < data.size()) {
      delay(20);
    }
  }

  ESP_LOGV(TAG, "Wrote %d bytes in %d chunks", data.size(), (data.size() + CHUNK_SIZE - 1) / CHUNK_SIZE);
}

void NavitasTAC::calculate_and_publish_speed_() {
  if (this->speed_sensor_ == nullptr) {
    return;
  }

  // Use controller values if available, otherwise fall back to configured defaults
  float tire_diameter = (this->controller_tire_diameter_ > 0) ? this->controller_tire_diameter_ : this->tire_diameter_;
  float rear_axle_ratio =
      (this->controller_rear_axle_ratio_ > 0) ? this->controller_rear_axle_ratio_ : this->rear_axle_ratio_;
  bool use_kilometers = this->vehicle_config_received_ ? this->controller_use_kilometers_ : this->use_kilometers_;

  // Speed calculation matching NavitasMotorControllerModel.js:726-742
  // Use absolute value of RPM for speed calculation (direction is handled by gear sensor)
  const float inchesPerMinuteToMPH = 60.0f * 2.0f * M_PI * 1.57828e-5f;
  float speed = inchesPerMinuteToMPH * (fabs(this->current_rpm_) * tire_diameter / 2.0f) / rear_axle_ratio;

  if (speed >= 0.0f) {
    // Convert to kilometers if configured
    if (use_kilometers) {
      speed = speed * 1.609344f;  // mph to km/h conversion
    }

    const char *config_source = this->vehicle_config_received_ ? "controller" : "config";
    ESP_LOGD(TAG, "Calculated speed: %.2f %s (RPM: %.1f, Tire: %.1f\", Ratio: %.1f) [%s]", speed,
             use_kilometers ? "km/h" : "mph", this->current_rpm_, tire_diameter, rear_axle_ratio, config_source);

    this->speed_sensor_->publish_state(speed);
  }
}

uint8_t NavitasTAC::get_command_for_address_(uint16_t address) {
  // Command selection logic from DeviceCommunication.cs
  if (address >= 768) {            // 0x300
    return CMD_READ_PARAMS_FLASH;  // 0x28 - Flash memory
  } else if (address >= 512) {     // 0x200
    return CMD_READ_PARAMS_HIGH;   // 0x26
  } else if (address >= 256) {     // 0x100
    return CMD_READ_PARAMS_MID;    // 0x23
  } else {
    return CMD_READ_PARAMS_LOW;  // 0x20 - RAM parameters
  }
}

}  // namespace navitas_tac
}  // namespace esphome
