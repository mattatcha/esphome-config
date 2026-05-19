#pragma once

#include "esphome/core/component.h"
#include "esphome/components/climate/climate.h"

namespace esphome {
namespace midea_xye {

class MideaXYE;  // forward decl

class MideaXYEClimate : public climate::Climate, public Component {
 public:
  void setup() override {}
  void dump_config() override;

  void set_parent(MideaXYE *p) { parent_ = p; }

  // Called by MideaXYE when a valid 32-byte 0xC0 status frame arrives.
  void on_status_frame(const uint8_t *data);

 protected:
  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;

  // Convert this->mode / this->fan_mode / this->target_temperature /
  // this->preset into wire-format bytes and push to parent.
  void push_desired_state_();

  MideaXYE *parent_{nullptr};
};

}  // namespace midea_xye
}  // namespace esphome
