#include "midea_xye_climate.h"
#include "midea_xye.h"
#include "esphome/core/log.h"

namespace esphome {
namespace midea_xye {

static const char *const TAG = "midea_xye.climate";

climate::ClimateTraits MideaXYEClimate::traits() {
  climate::ClimateTraits t;
  t.set_supports_current_temperature(true);
  t.set_supports_two_point_target_temperature(false);
  t.set_supported_modes({
      climate::CLIMATE_MODE_OFF,
      climate::CLIMATE_MODE_COOL,
      climate::CLIMATE_MODE_HEAT,
      climate::CLIMATE_MODE_DRY,
      climate::CLIMATE_MODE_FAN_ONLY,
      climate::CLIMATE_MODE_HEAT_COOL,  // = AUTO
  });
  t.set_supported_fan_modes({
      climate::CLIMATE_FAN_AUTO,
      climate::CLIMATE_FAN_LOW,
      climate::CLIMATE_FAN_MEDIUM,
      climate::CLIMATE_FAN_HIGH,
  });
  // Turbo is reported via byte[19] bit 4 of the C0 status frame.
  t.set_supported_presets({
      climate::CLIMATE_PRESET_NONE,
      climate::CLIMATE_PRESET_BOOST,
  });
  // Setpoint range matches what the controller's encoding can reach
  // (raw byte 0x40..0x7F → 0..63 °C); narrow to a sensible HVAC range.
  t.set_visual_min_temperature(16);
  t.set_visual_max_temperature(30);
  t.set_visual_temperature_step(1);
  return t;
}

void MideaXYEClimate::dump_config() {
  LOG_CLIMATE("", "Midea XYE Climate", this);
}

void MideaXYEClimate::on_status_frame(const uint8_t *data) {
  // Layout verified on-wire — see midea_xye.h header comment for offsets.
  const uint8_t mode_b = data[8];
  const uint8_t fan_b  = data[9];
  const uint8_t set_b  = data[10];
  const uint8_t t1_b   = data[11];  // AC's intake-air sensor

  // Mode: bit 7 = power. When off, byte holds the remembered mode in the low
  // bits (we don't reflect that to HA — HA just sees OFF).
  if ((mode_b & 0x80) == 0) {
    this->mode = climate::CLIMATE_MODE_OFF;
  } else {
    switch (mode_b & 0x7F) {
      case 0x00: this->mode = climate::CLIMATE_MODE_HEAT_COOL; break;  // AUTO
      case 0x08: this->mode = climate::CLIMATE_MODE_COOL;      break;
      case 0x04: this->mode = climate::CLIMATE_MODE_HEAT;      break;
      case 0x02: this->mode = climate::CLIMATE_MODE_DRY;       break;
      case 0x01: this->mode = climate::CLIMATE_MODE_FAN_ONLY;  break;
      default:   this->mode = climate::CLIMATE_MODE_OFF;       break;
    }
  }

  // Fan: bit 7 = AUTO overlay; low nibble = current speed. We map the
  // *intent* — if AUTO is requested, report AUTO regardless of current speed.
  if (fan_b & 0x80) {
    this->fan_mode = climate::CLIMATE_FAN_AUTO;
  } else {
    switch (fan_b & 0x0F) {
      case 0x01: this->fan_mode = climate::CLIMATE_FAN_HIGH;   break;
      case 0x02: this->fan_mode = climate::CLIMATE_FAN_MEDIUM; break;
      case 0x04: this->fan_mode = climate::CLIMATE_FAN_LOW;    break;
      default:   this->fan_mode.reset();                       break;
    }
  }

  // Setpoint byte encoding depends on the AC's display-mode setting.
  // °F mode: raw decimal °F. °C mode: raw decimal °C with bit 6 optionally
  // set as a status flag. Mask both bits 6+7 for °C, mask only bit 7 (legacy
  // °F-mode flag) for °F.
  if (this->parent_ != nullptr && this->parent_->is_fahrenheit()) {
    const int f = static_cast<int>(set_b & 0x7F);
    this->target_temperature = (static_cast<float>(f) - 32.0f) * 5.0f / 9.0f;
  } else {
    this->target_temperature = static_cast<float>(set_b & 0x3F);
  }
  this->current_temperature = (static_cast<int>(t1_b) - 40) * 0.5f;

  // Turbo / BOOST preset reflected from byte[19] bit 4.
  this->preset = (data[19] & 0x10) ? climate::CLIMATE_PRESET_BOOST
                                   : climate::CLIMATE_PRESET_NONE;

  this->publish_state();
}

void MideaXYEClimate::control(const climate::ClimateCall &call) {
  if (call.get_mode())               this->mode               = *call.get_mode();
  if (call.get_fan_mode())           this->fan_mode           = *call.get_fan_mode();
  if (call.get_target_temperature()) this->target_temperature = *call.get_target_temperature();
  if (call.get_preset())             this->preset             = *call.get_preset();

  this->push_desired_state_();
  this->publish_state();
}

void MideaXYEClimate::push_desired_state_() {
  if (this->parent_ == nullptr) return;

  // Mode byte = (power_bit) | mode_enum_bits. Power bit 0x80 is cleared when
  // CLIMATE_MODE_OFF; otherwise set, with the lower bits chosen per the mode.
  uint8_t mode_b = 0x00;
  switch (this->mode) {
    case climate::CLIMATE_MODE_OFF:        mode_b = 0x00;            break;
    case climate::CLIMATE_MODE_HEAT_COOL:  mode_b = 0x80 | 0x00;     break;  // AUTO
    case climate::CLIMATE_MODE_COOL:       mode_b = 0x80 | 0x08;     break;
    case climate::CLIMATE_MODE_HEAT:       mode_b = 0x80 | 0x04;     break;
    case climate::CLIMATE_MODE_DRY:        mode_b = 0x80 | 0x02;     break;
    case climate::CLIMATE_MODE_FAN_ONLY:   mode_b = 0x80 | 0x01;     break;
    default:                               mode_b = 0x00;            break;
  }

  // Fan byte: HIGH/MED/LOW use lower bits, AUTO sets bit 7.
  uint8_t fan_b = 0x80;  // default AUTO
  if (this->fan_mode.has_value()) {
    switch (*this->fan_mode) {
      case climate::CLIMATE_FAN_AUTO:   fan_b = 0x80; break;
      case climate::CLIMATE_FAN_HIGH:   fan_b = 0x01; break;
      case climate::CLIMATE_FAN_MEDIUM: fan_b = 0x02; break;
      case climate::CLIMATE_FAN_LOW:    fan_b = 0x04; break;
      default:                          fan_b = 0x80; break;
    }
  }

  // Setpoint TX encoding mirrors the AC's display-mode setting. °C mode:
  // raw byte = °C + 0x40. °F mode: based on captured °F-mode controller TX,
  // raw byte = °F | 0x80. Clamp to a sensible HVAC range first.
  float sp_c = this->target_temperature;
  if (sp_c < 16.0f) sp_c = 16.0f;
  if (sp_c > 30.0f) sp_c = 30.0f;
  uint8_t setpoint_b;
  if (this->parent_ != nullptr && this->parent_->is_fahrenheit()) {
    const int sp_f = static_cast<int>(sp_c * 9.0f / 5.0f + 32.0f + 0.5f);  // round
    setpoint_b = static_cast<uint8_t>(0x80 | (sp_f & 0x7F));
  } else {
    setpoint_b = static_cast<uint8_t>(0x40 + static_cast<int>(sp_c));
  }

  const bool turbo = this->preset.has_value() && *this->preset == climate::CLIMATE_PRESET_BOOST;

  this->parent_->set_desired_state(mode_b, fan_b, setpoint_b, turbo);
}

}  // namespace midea_xye
}  // namespace esphome
