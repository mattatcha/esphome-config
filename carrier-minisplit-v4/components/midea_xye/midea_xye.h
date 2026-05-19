#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace midea_xye {

class MideaXYEClimate;  // forward decl, defined in midea_xye_climate.h

// Carrier/Midea XYE wired-controller bus.
//
// Phase 1: passive sniffer. We never drive TX. The class buffers bytes from
// the UART, hunts for a 0xAA start, and tries to close the frame at either
// 16 or 32 bytes (the two legal lengths). A frame is accepted only if the
// trailer is 0x55 AND the simple-sum checksum at position [len-2] matches.
//
// Byte layout (verified from on-wire captures with 17 → 30 → 17 °C setpoint sweep):
//   [0]      0xAA           start
//   [1]      type           0xC4 = controller → IDU set, 0xC0 = IDU → controller status,
//                           0xC3 = ?(boot), 0xC6 = ?(boot)
//   [len-2]  checksum       0xFF - (sum of every other byte & 0xFF)
//   [len-1]  0x55           end
//
// 32-byte AC status reply (type 0xC0) — VERIFIED on-wire:
//   frame[7]   address/unit-id  constant 0x14 in every capture (NOT a temperature)
//   frame[8]   mode             bit 7 = power; lower bits: 0x00 AUTO / 0x08 COOL /
//                                0x04 HEAT / 0x02 DRY / 0x01 FAN_ONLY. Byte=0x00 → OFF
//   frame[9]   fan              bit 7 = AUTO overlay; 0x01 HIGH / 0x02 MED / 0x04 LOW
//   frame[10]  setpoint         raw - 0x40 = °C  (17 °C → 0x51, 30 °C → 0x5E)
//   frame[11]  T1 indoor air    (raw - 40) / 2  °C  (verified: hair-dryer 20→55 °C test)
//   frame[12]  T2 indoor coil   (raw - 40) / 2  °C  (drops as AC cools harder)
//   frame[14]  T3 outdoor coil  (raw - 40) / 2  °C  (rises with compressor load —
//                                ≈ ambient at idle, ambient + ΔT under load)
//   frame[19]  flag/state bits  bit 4 (0x10) = turbo mode active (verified by toggle test).
//                                bits 0,1 (0x03) appear set during normal operation.
//                                Other bits TBD.
//
// T4 (outdoor ambient air, distinct from condenser coil) is not exposed in C0.
// Possibly returned via a different frame type the controller issues only when
// the service menu is open, or simply not measured on this AC variant.
//
// 32-byte controller set (type 0xC4) — VERIFIED:
//   frame[15]  control flags    bit 3 (0x08) = turbo requested. Baseline 0x20.
//   frame[16]  mode
//   frame[17]  fan
//   frame[18]  setpoint         (same °C + 0x40 encoding)
//
// 16-byte short frames (both 0xC4 and 0xC0): poll/ack, no payload of interest;
// the only data byte that varies is [13] = 0x3B (poll) or 0x3F (ack).
class MideaXYE : public Component, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Sink registration (called from platform .py files).
  void set_t1_sensor(sensor::Sensor *s) { t1_sensor_ = s; }
  void set_t2_sensor(sensor::Sensor *s) { t2_sensor_ = s; }
  void set_t3_sensor(sensor::Sensor *s) { t3_sensor_ = s; }
  void set_fan_speed_text_sensor(text_sensor::TextSensor *s) { fan_speed_text_sensor_ = s; }
  void set_climate(MideaXYEClimate *c) { climate_ = c; }

  // Phase 3: TX path. Disabled by default — only the wired controller should be
  // the master on the bus. Enable explicitly (and physically unplug the original
  // controller first) via the YAML config or the runtime button below.
  void set_tx_enabled(bool en) { tx_enabled_ = en; }
  bool is_tx_enabled() const { return tx_enabled_; }

  // Runtime trigger: kick off the boot handshake (C3 + C6) and start the
  // periodic C4 poll/set cycle. Safe to call multiple times — resets the state
  // machine each time. Exposed as a template button in the YAML so the user can
  // start TX precisely when an API client is attached and watching logs.
  void start_handshake_and_poll();
  void stop_tx();

  // Called by MideaXYEClimate::control() to update the desired state that the
  // next periodic TX tick will encode into a C4 set frame. mode_byte is the
  // *complete* wire byte (power_bit | mode_bits); 0x00 is special-cased as
  // "OFF but keep remembered mode bits unchanged".
  void set_desired_state(uint8_t mode_byte, uint8_t fan_byte,
                         uint8_t setpoint_byte, bool turbo);

  // Helper used internally and by the boot frames: compose the wire mode byte
  // from desired_power_ + desired_mode_bits_.
  uint8_t wire_mode_byte_() const {
    return (this->desired_power_ ? 0x80 : 0x00) | this->desired_mode_bits_;
  }

  // Fires a single C4-set frame onto the bus right now, regardless of
  // tx_enabled. For the "does the controller update from the AC echo?"
  // experiment — collision risk is low for a one-shot frame compared to
  // continuous polling.
  void send_one_shot_set(uint8_t mode_byte, uint8_t fan_byte,
                         uint8_t setpoint_byte, bool turbo);

 protected:
  static constexpr uint8_t START_BYTE = 0xAA;
  static constexpr uint8_t END_BYTE = 0x55;
  static constexpr size_t SHORT_LEN = 16;
  static constexpr size_t LONG_LEN = 32;
  static constexpr size_t MAX_LEN = LONG_LEN;
  // The AC has been observed transmitting its C5 frames in 3-byte bursts with
  // multi-tens-of-ms gaps between bursts (likely a small TX FIFO inside the AC).
  // Set the timeout generously so we reassemble across those gaps; the trade-off
  // is that detecting end-of-frame takes ~80 ms instead of ~15 ms.
  static constexpr uint32_t INTER_BYTE_TIMEOUT_MS = 80;
  static constexpr size_t MIN_FRAME_LEN = 6;  // for loose-close: smallest plausible frame

  uint8_t buf_[MAX_LEN]{};
  size_t pos_{0};
  uint32_t last_byte_ms_{0};

  void reset_();
  void feed_(uint8_t b);
  // Returns true if pos_ matches a valid (length, trailer, checksum) frame.
  bool frame_complete_at_(size_t len) const;
  // Scan the partial buffer for ANY valid (trailer + checksum) close ≥ MIN_FRAME_LEN.
  // Used on inter-byte timeout to salvage variable-length frames.
  bool try_loose_close_();
  void on_frame_(const uint8_t *data, size_t len);
  // Publish parsed fields from a 32-byte 0xC0 status frame to registered sinks.
  void publish_status_(const uint8_t *data);

  static uint8_t calc_checksum_(const uint8_t *data, size_t len);
  static const char *mode_name_(uint8_t b);
  static const char *fan_name_(uint8_t b);
  static int decode_setpoint_c_(uint8_t raw);
  // Temperature encoding for T1/T2/T4 (and presumed T3): (raw - 40) / 2 °C.
  // Verified against hair-dryer T1 test: raw 75→17.5 °C, raw 118→39 °C.
  static float decode_temp_c_(uint8_t raw);

  sensor::Sensor *t1_sensor_{nullptr};
  sensor::Sensor *t2_sensor_{nullptr};
  sensor::Sensor *t3_sensor_{nullptr};
  text_sensor::TextSensor *fan_speed_text_sensor_{nullptr};
  MideaXYEClimate *climate_{nullptr};

  // -------- TX path --------
  static constexpr uint32_t TX_CYCLE_MS = 1000;  // full cycle (poll + set + wait)
  static constexpr uint32_t TX_POLL_TO_SET_MS = 100;  // gap between poll and set
  // Listen-before-talk: don't transmit unless the bus has been idle (no RX
  // bytes) for at least this long. The AC's response to our last frame can
  // arrive 40-100 ms after we start TXing — without this gate our subsequent
  // frames collide with its tail.
  static constexpr uint32_t TX_LINE_QUIET_MS = 60;

  bool tx_enabled_{false};
  // Runtime gate (set true either by tx_enabled_ at setup time or by the
  // start_handshake_and_poll() button at runtime). loop() checks THIS instead
  // of tx_enabled_ so a button press takes effect even if the YAML disabled TX.
  bool tx_running_{false};
  // The wired controller never sends mode=0x00 on the wire — even when the AC
  // is off it carries the *remembered* mode (e.g. 0x08 = COOL without power
  // bit). We track the remembered low-7 bits separately and OR in the power
  // bit when constructing the wire mode byte.
  uint8_t desired_mode_bits_{0x08};  // default: remembered as COOL
  bool desired_power_{false};
  uint8_t desired_fan_{0x80};
  uint8_t desired_setpoint_{0x40 + 22};  // = 0x56 = 22 °C
  bool desired_turbo_{false};

  // Last 0xC0 status frame received from the AC. The controller's C3-long
  // wake frame is essentially this status echoed back with byte[1] swapped
  // to 0xC3 and byte[8] forced to 0x00. We cache the last received C0 here
  // so we can build a proper C3-long during boot.
  uint8_t last_c0_status_[LONG_LEN]{};
  bool have_c0_status_{false};

  // Set when the user has changed any field of desired_*. The next post-boot
  // TX cycle will fire a C3-short to commit the change before the next
  // C4-poll/set. C4-set on its own appears to be "what I'm tracking" reporting;
  // C3-short is the actual "do this" command per captured controller behavior.
  bool state_dirty_{false};

  uint32_t last_cycle_ms_{0};
  uint8_t tx_phase_{0};  // 0 = idle (waiting for cycle), 1 = poll sent, 2 = set sent

  // Boot handshake: replicates the full sequence the wired controller sends
  // at power-on, observed in raw captures:
  //   phase 0: bus-break preamble (single 0x00 byte)
  //   phase 1: wait ~1700 ms, then C4-poll  (heartbeat)
  //   phase 2: wait ~80 ms,   then C4-set   (state push)
  //   phase 3: wait ~70 ms,   then C0-poll  (request status)  ← AC replies here
  //   phase 4: wait ~150 ms,  then C3-short (state command, gives AC time to reply)
  //   phase 5: wait ~80 ms,   then C3-long  (status echo mirror)
  //   phase 6: wait ~70 ms,   then C6-short (identify)
  //   phase 7: wait ~90 ms,   then C6-long  (capability echo)
  // After phase 7, the regular C4 poll → C4-set → C0-poll cycle takes over.
  static constexpr uint32_t BOOT_DELAY_MS = 200;  // delay before phase 0 fires
  static constexpr uint8_t BOOT_PHASE_COUNT = 8;
  bool boot_done_{false};
  uint32_t boot_start_ms_{0};
  uint8_t boot_phase_{0};
  uint32_t last_boot_action_ms_{0};

  void tx_pump_();
  void send_break_();
  void send_c4_poll_();
  void send_c4_set_();
  void send_c0_poll_();   // master → AC: "give me your status"
  void send_c3_short_();
  void send_c3_long_();
  void send_c6_short_();
  void send_c6_long_();
};

}  // namespace midea_xye
}  // namespace esphome
