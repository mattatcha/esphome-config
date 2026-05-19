#include "midea_xye.h"
#include "midea_xye_climate.h"

#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace midea_xye {

static const char *const TAG = "midea_xye";

void MideaXYE::setup() {
  this->reset_();
  ESP_LOGW(TAG, "midea_xye setup: tx_enabled=%s", this->tx_enabled_ ? "TRUE" : "false");
  if (this->tx_enabled_) {
    this->start_handshake_and_poll();
  }
}

void MideaXYE::start_handshake_and_poll() {
  ESP_LOGW(TAG, "start_handshake_and_poll: resetting state and scheduling boot");
  this->boot_done_ = false;
  this->boot_phase_ = 0;
  this->boot_start_ms_ = millis() + BOOT_DELAY_MS;
  this->last_boot_action_ms_ = 0;
  this->tx_phase_ = 0;
  this->last_cycle_ms_ = 0;
  this->tx_running_ = true;
}

void MideaXYE::stop_tx() {
  ESP_LOGW(TAG, "stop_tx: halting periodic TX");
  this->tx_running_ = false;
}

void MideaXYE::dump_config() {
  ESP_LOGCONFIG(TAG, "Midea XYE:");
  ESP_LOGCONFIG(TAG, "  TX: %s", this->tx_enabled_ ? "ENABLED (master mode)" : "disabled (sniffer)");
  this->check_uart_settings(4800);
}

void MideaXYE::loop() {
  const uint32_t now = millis();
  // Reset a partial frame if the line has gone idle mid-frame. Before dropping,
  // try to salvage a valid frame at some length other than 16/32 — variable-length
  // frames (e.g. the 22-byte 0xC0 reply form documented in v2's Frame.h) exist.
  if (this->pos_ > 0 && (now - this->last_byte_ms_) > INTER_BYTE_TIMEOUT_MS) {
    if (this->try_loose_close_()) {
      this->reset_();
    } else {
      char hex[3 * MAX_LEN + 1];
      for (size_t i = 0; i < this->pos_; i++) {
        snprintf(&hex[i * 3], 4, "%02X ", this->buf_[i]);
      }
      hex[this->pos_ * 3 - 1] = '\0';
      ESP_LOGW(TAG, "inter-byte timeout, dropping %u partial bytes: %s",
               (unsigned) this->pos_, hex);
      this->reset_();
    }
  }

  // Drain everything available, accumulate into raw_chunk_, then log on idle.
  static uint8_t raw_chunk[64];
  static size_t raw_pos = 0;
  static uint32_t raw_last_ms = 0;

  while (this->available()) {
    uint8_t b;
    if (!this->read_byte(&b)) break;
    this->last_byte_ms_ = now;
    raw_last_ms = now;
    if (raw_pos < sizeof(raw_chunk)) raw_chunk[raw_pos++] = b;
    this->feed_(b);
  }

  // Flush the raw chunk when the line has been idle for a moment, or when full.
  if (raw_pos > 0 && (raw_pos == sizeof(raw_chunk) || (now - raw_last_ms) >= 50)) {
    char hex[3 * sizeof(raw_chunk) + 1];
    for (size_t i = 0; i < raw_pos; i++) snprintf(&hex[i * 3], 4, "%02X ", raw_chunk[i]);
    hex[raw_pos * 3 - 1] = '\0';
    ESP_LOGI(TAG, "RAW RX (%u): %s", (unsigned) raw_pos, hex);
    raw_pos = 0;
  }

  if (this->tx_running_) this->tx_pump_();
}

void MideaXYE::reset_() {
  this->pos_ = 0;
}

void MideaXYE::feed_(uint8_t b) {
  // Resync on START_BYTE: anything before it is noise.
  if (this->pos_ == 0) {
    if (b != START_BYTE) return;
  }

  if (this->pos_ >= MAX_LEN) {
    ESP_LOGW(TAG, "buffer overrun without trailer, resyncing");
    this->reset_();
    if (b != START_BYTE) return;
  }

  this->buf_[this->pos_++] = b;

  // Try to close as a 16-byte short frame first.
  if (this->pos_ == SHORT_LEN && this->frame_complete_at_(SHORT_LEN)) {
    this->on_frame_(this->buf_, SHORT_LEN);
    this->reset_();
    return;
  }

  // If we've reached 32, it must close here.
  if (this->pos_ == LONG_LEN) {
    if (this->frame_complete_at_(LONG_LEN)) {
      this->on_frame_(this->buf_, LONG_LEN);
    } else {
      char hex[3 * LONG_LEN + 1];
      for (size_t i = 0; i < LONG_LEN; i++) {
        snprintf(&hex[i * 3], 4, "%02X ", this->buf_[i]);
      }
      hex[LONG_LEN * 3 - 1] = '\0';
      ESP_LOGW(TAG, "32-byte frame failed validation (trailer=0x%02X chk=0x%02X exp=0x%02X): %s",
               this->buf_[LONG_LEN - 1], this->buf_[LONG_LEN - 2],
               calc_checksum_(this->buf_, LONG_LEN), hex);
    }
    this->reset_();
  }
}

bool MideaXYE::frame_complete_at_(size_t len) const {
  if (this->pos_ != len) return false;
  if (this->buf_[len - 1] != END_BYTE) return false;
  return this->buf_[len - 2] == calc_checksum_(this->buf_, len);
}

uint8_t MideaXYE::calc_checksum_(const uint8_t *data, size_t len) {
  // Outer checksum: 0xFF - (sum of all bytes except the CRC byte itself & 0xFF).
  // CRC byte sits at index [len-2]; trailer at [len-1] is included in the sum.
  uint32_t sum = 0;
  for (size_t i = 0; i < len; i++) {
    if (i == len - 2) continue;
    sum += data[i];
  }
  return static_cast<uint8_t>(0xFF - (sum & 0xFF));
}

void MideaXYE::on_frame_(const uint8_t *data, size_t len) {
  // Always log hex first — protocol-decode below is speculative, raw bytes are ground truth.
  char hex[3 * MAX_LEN + 1];
  for (size_t i = 0; i < len; i++) {
    snprintf(&hex[i * 3], 4, "%02X ", data[i]);
  }
  hex[len * 3 - 1] = '\0';

  const uint8_t type = data[1];
  // Master-direction frames (C4, C3, C6, and 16-byte C0) come from the
  // controller — that's us when we're the master, or our TX echoed back via
  // transceiver loopback. The AC only ever emits the 32-byte AA C0 status.
  const char *dir = (type == 0xC4 && len == SHORT_LEN) ? "RX C4-poll" :
                    (type == 0xC4 && len == LONG_LEN)  ? "RX C4-set"  :
                    (type == 0xC0 && len == SHORT_LEN) ? "RX C0-poll" :
                    (type == 0xC0 && len == LONG_LEN)  ? "RX C0-status (AC)" :
                    (type == 0xC3)                     ? "RX C3-boot"   :
                    (type == 0xC6)                     ? "RX C6-boot"   :
                                                         "RX unknown";

  ESP_LOGI(TAG, "[%s len=%u] %s", dir, (unsigned) len, hex);

  if (len == LONG_LEN && (type == 0xC0 || type == 0xC3)) {
    // AC status reply (C0) and controller wake-up echo (C3) share the same
    // 32-byte payload layout. Verified on-wire via hair-dryer T1 test plus
    // setpoint and fan sweeps.
    const uint8_t mode_b = data[8];
    const uint8_t fan_b  = data[9];
    const uint8_t set_b  = data[10];
    const uint8_t t1_b   = data[11];
    const uint8_t t2_b   = data[12];
    const uint8_t t3_b   = data[14];
    const uint8_t flags  = data[19];

    ESP_LOGI(TAG,
             "  status: power=%s mode=%s(0x%02X) fan=%s(0x%02X) "
             "set=%dC(0x%02X) T1=%.1fC T2=%.1fC T3=%.1fC flags=0x%02X",
             (mode_b & 0x80) ? "ON" : "off",
             mode_name_(mode_b), mode_b,
             fan_name_(fan_b), fan_b,
             decode_setpoint_c_(set_b), set_b,
             decode_temp_c_(t1_b),
             decode_temp_c_(t2_b),
             decode_temp_c_(t3_b),
             flags);

    // Only publish from genuine 0xC0 (the AC). C3 is the controller's own
    // wake-up echo and can carry transitional values that aren't the AC's truth.
    if (type == 0xC0) this->publish_status_(data);
  } else if (len == LONG_LEN && (type == 0xC4 || type == 0xC6)) {
    // Controller set (C4) and controller identify-echo (C6) share the same
    // 32-byte layout: mode/fan/setpoint at +8 offset vs C0 status.
    const uint8_t mode_b = data[16];
    const uint8_t fan_b  = data[17];
    const uint8_t set_b  = data[18];
    ESP_LOGI(TAG, "  ctrl-tx: power=%s mode=%s(0x%02X) fan=%s(0x%02X) set=%dC(0x%02X)",
             (mode_b & 0x80) ? "ON" : "off",
             mode_name_(mode_b), mode_b,
             fan_name_(fan_b), fan_b,
             decode_setpoint_c_(set_b), set_b);
  } else if (len == SHORT_LEN && type == 0xC3) {
    // Controller short wakeup carries mode/fan/setpoint at bytes 6/7/8.
    const uint8_t mode_b = data[6];
    const uint8_t fan_b  = data[7];
    const uint8_t set_b  = data[8];
    ESP_LOGI(TAG, "  c3-wake: power=%s mode=%s(0x%02X) fan=%s(0x%02X) set=%dC(0x%02X)",
             (mode_b & 0x80) ? "ON" : "off",
             mode_name_(mode_b), mode_b,
             fan_name_(fan_b), fan_b,
             decode_setpoint_c_(set_b), set_b);
  }
}

const char *MideaXYE::mode_name_(uint8_t b) {
  // Bit 7 (0x80) is the power-on flag. Lower bits encode mode.
  // When power is off but a mode is remembered, byte = (mode & 0x7F).
  // Pure 0x00 = OFF with no mode preference (treated specially).
  if (b == 0x00) return "OFF";
  switch (b & 0x7F) {
    case 0x00: return "AUTO";  // i.e. b == 0x80
    case 0x08: return "COOL";
    case 0x04: return "HEAT";
    case 0x02: return "DRY";
    case 0x01: return "FAN_ONLY";
    default:   return "?";
  }
}

const char *MideaXYE::fan_name_(uint8_t b) {
  // Bit 7 = "AUTO" flag. Lower bits = actual current speed bitmap:
  // 0x01 HIGH, 0x02 MED, 0x04 LOW. When AUTO is on, AC echoes back
  // both bits (e.g. 0x81 = AUTO currently spinning HIGH).
  const bool is_auto = (b & 0x80) != 0;
  switch (b & 0x0F) {
    case 0x00: return is_auto ? "AUTO" : "?";
    case 0x01: return is_auto ? "AUTO/HIGH" : "HIGH";
    case 0x02: return is_auto ? "AUTO/MED"  : "MED";
    case 0x04: return is_auto ? "AUTO/LOW"  : "LOW";
    default:   return "?";
  }
}

int MideaXYE::decode_setpoint_c_(uint8_t raw) {
  // The controller switches encoding based on its display unit:
  //   °C mode (high bit clear): raw - 0x40 = °C       (e.g. 17 °C → 0x51, 30 °C → 0x5E)
  //   °F mode (high bit set):   (raw & 0x7F) °F → °C  (e.g. 69 °F → 0xC5, 86 °F → 0xD6)
  if (raw & 0x80) {
    const int f = static_cast<int>(raw & 0x7F);
    return (f - 32) * 5 / 9;
  }
  return static_cast<int>(raw) - 0x40;
}

float MideaXYE::decode_temp_c_(uint8_t raw) {
  // Half-degree Celsius with -20 °C zero. raw 80 -> 20 °C, raw 118 -> 39 °C.
  // Verified by hair-dryer test on T1: 0x4B (75) ↔ 17 °C, 0x76 (118) ↔ 39 °C
  // matches user-displayed values within 0.5 °C.
  return (static_cast<int>(raw) - 40) * 0.5f;
}

void MideaXYE::set_desired_state(uint8_t mode_byte, uint8_t fan_byte,
                                 uint8_t setpoint_byte, bool turbo) {
  // Snapshot to detect whether anything actually changed.
  const uint8_t prev_mode_bits = this->desired_mode_bits_;
  const bool    prev_power     = this->desired_power_;
  const uint8_t prev_fan       = this->desired_fan_;
  const uint8_t prev_set       = this->desired_setpoint_;
  const bool    prev_turbo     = this->desired_turbo_;

  // The wired controller never sends mode=0x00 on the wire; even when the AC
  // is off it carries the remembered mode bits (e.g. 0x08 = COOL no power).
  // We split the incoming wire byte into (power, mode_bits) and update each
  // independently. mode_byte=0x00 from HA means "OFF with no change to the
  // remembered mode bits".
  if (mode_byte == 0x00) {
    this->desired_power_ = false;
    // keep this->desired_mode_bits_ unchanged
  } else {
    this->desired_power_     = (mode_byte & 0x80) != 0;
    this->desired_mode_bits_ = mode_byte & 0x7F;
  }
  this->desired_fan_      = fan_byte;
  this->desired_setpoint_ = setpoint_byte;
  this->desired_turbo_    = turbo;

  const bool changed = prev_mode_bits != this->desired_mode_bits_ ||
                       prev_power     != this->desired_power_ ||
                       prev_fan       != this->desired_fan_ ||
                       prev_set       != this->desired_setpoint_ ||
                       prev_turbo     != this->desired_turbo_;
  if (changed) this->state_dirty_ = true;

  ESP_LOGI(TAG,
           "desired state: power=%s mode_bits=0x%02X (wire=0x%02X) "
           "fan=0x%02X set=0x%02X(%dC) turbo=%d%s",
           this->desired_power_ ? "ON" : "off",
           this->desired_mode_bits_, this->wire_mode_byte_(),
           fan_byte, setpoint_byte, setpoint_byte - 0x40, turbo,
           changed ? " [will commit via C3-short next cycle]" : "");
}

void MideaXYE::send_one_shot_set(uint8_t mode_byte, uint8_t fan_byte,
                                 uint8_t setpoint_byte, bool turbo) {
  // Stash current desired-state, temporarily overwrite, send, restore.
  const uint8_t prev_bits  = this->desired_mode_bits_;
  const bool    prev_power = this->desired_power_;
  const uint8_t prev_fan   = this->desired_fan_;
  const uint8_t prev_set   = this->desired_setpoint_;
  const bool    prev_turbo = this->desired_turbo_;

  if (mode_byte == 0x00) {
    this->desired_power_ = false;
  } else {
    this->desired_power_     = (mode_byte & 0x80) != 0;
    this->desired_mode_bits_ = mode_byte & 0x7F;
  }
  this->desired_fan_      = fan_byte;
  this->desired_setpoint_ = setpoint_byte;
  this->desired_turbo_    = turbo;

  ESP_LOGW(TAG, "one-shot C4-set: mode=0x%02X fan=0x%02X set=0x%02X(%dC) turbo=%d",
           this->wire_mode_byte_(), fan_byte, setpoint_byte,
           setpoint_byte - 0x40, turbo);
  this->send_c4_set_();

  this->desired_mode_bits_ = prev_bits;
  this->desired_power_     = prev_power;
  this->desired_fan_       = prev_fan;
  this->desired_setpoint_  = prev_set;
  this->desired_turbo_     = prev_turbo;
}

void MideaXYE::tx_pump_() {
  const uint32_t now = millis();

  // Listen-before-talk: if we just received bytes, don't transmit yet. The AC
  // may still be in the middle of responding to our previous frame.
  if (this->last_byte_ms_ != 0 &&
      (now - this->last_byte_ms_) < TX_LINE_QUIET_MS) {
    return;
  }

  if (!this->boot_done_) {
    // Per-phase delay: how long to wait after the previous boot action (or
    // after boot_start_ms_ for phase 0) before firing this phase. Values are
    // taken from the captured wired-controller startup.
    static const uint32_t PHASE_DELAY_MS[BOOT_PHASE_COUNT] = {
      0,       // 0 — send bus-break preamble
      1700,    // 1 — C4-poll  (controller waits ~1.7 s after break)
      80,      // 2 — C4-set
      70,      // 3 — C0-poll  (master asks AC for status)
      150,     // 4 — C3-short (gives AC ~150 ms to send C0-status before this)
      80,      // 5 — C3-long
      70,      // 6 — C6-short
      90,      // 7 — C6-long
    };

    if (this->boot_phase_ == 0 && now < this->boot_start_ms_) return;

    const uint32_t reference = (this->boot_phase_ == 0)
                                   ? this->boot_start_ms_
                                   : this->last_boot_action_ms_;
    if (now - reference < PHASE_DELAY_MS[this->boot_phase_]) return;

    if (this->boot_phase_ == 0) {
      ESP_LOGW(TAG, "boot path entered at millis()=%u", (unsigned) now);
    }

    switch (this->boot_phase_) {
      case 0: this->send_break_();     break;
      case 1: this->send_c4_poll_();   break;
      case 2: this->send_c4_set_();    break;
      case 3: this->send_c0_poll_();   break;
      case 4: this->send_c3_short_();  break;
      case 5: this->send_c3_long_();   break;
      case 6: this->send_c6_short_();  break;
      case 7: this->send_c6_long_();   break;
    }

    this->last_boot_action_ms_ = now;
    this->boot_phase_++;
    if (this->boot_phase_ >= BOOT_PHASE_COUNT) {
      this->boot_done_ = true;
      this->last_cycle_ms_ = now;
      ESP_LOGI(TAG, "boot sequence complete; starting C4 poll/set cycle");
    }
    return;
  }

  switch (this->tx_phase_) {
    case 0:  // idle — start a new cycle when interval elapsed
      if (now - this->last_cycle_ms_ < TX_CYCLE_MS) return;
      this->last_cycle_ms_ = now;
      if (this->state_dirty_) {
        this->send_c3_short_();
        this->state_dirty_ = false;
        // Don't advance tx_phase_; this cycle is just the commit. Next cycle
        // (1 s later) resumes the normal poll/set/status sequence.
      } else {
        this->send_c4_poll_();
        this->tx_phase_ = 1;
      }
      break;
    case 1:  // poll has been sent; wait, then send set
      if (now - this->last_cycle_ms_ < TX_POLL_TO_SET_MS) return;
      this->send_c4_set_();
      this->tx_phase_ = 2;
      break;
    case 2:  // set has been sent; wait briefly, then ask AC for status
      if (now - this->last_cycle_ms_ < TX_POLL_TO_SET_MS + 80) return;
      this->send_c0_poll_();
      this->tx_phase_ = 0;  // back to idle until next cycle
      break;
  }
}

void MideaXYE::send_break_() {
  // Captured-controller preamble: a single 0x00 byte before the first real
  // frame. Sending a UART 0x00 puts the line low for the start bit + 8 zero
  // data bits = 9 bit-times at 4800 baud = ~1.88 ms of LOW, then HIGH for the
  // stop bit. Acts as a wake-up / bus-takeover signal for slaves.
  const uint8_t b = 0x00;
  this->write_byte(b);
  ESP_LOGI(TAG, "TX BREAK (0x00 preamble)");
}

void MideaXYE::send_c4_poll_() {
  // 16-byte master "heartbeat" poll. No fields to vary.
  static const uint8_t POLL[16] = {
    0xAA, 0xC4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x3B, 0x01, 0x55,
  };
  this->write_array(POLL, sizeof(POLL));
  ESP_LOGI(TAG, "TX C4-poll");
}

void MideaXYE::send_c0_poll_() {
  // 16-byte master "give me your status" poll. Same structure as the C4-poll
  // but with type=0xC0 and byte[13]=0x3F (vs 0x3B). The AC replies with a
  // 32-byte AA C0 status frame ~75 ms later — this is the ONLY way we get
  // real sensor data back.
  static const uint8_t POLL[16] = {
    0xAA, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x01, 0x55,
  };
  this->write_array(POLL, sizeof(POLL));
  ESP_LOGI(TAG, "TX C0-poll (request status)");
}

bool MideaXYE::try_loose_close_() {
  // Walk the buffer from the longest plausible frame down, looking for any
  // length where buf[len-1] == 0x55 and buf[len-2] matches our checksum rule.
  // Frames must start at index 0 with 0xAA (already guaranteed by feed_).
  for (size_t len = this->pos_; len >= MIN_FRAME_LEN; len--) {
    if (this->buf_[len - 1] != END_BYTE) continue;
    if (this->buf_[len - 2] != calc_checksum_(this->buf_, len)) continue;
    ESP_LOGI(TAG, "loose-close salvaged %u-byte frame (buffer had %u bytes)",
             (unsigned) len, (unsigned) this->pos_);
    this->on_frame_(this->buf_, len);
    return true;
  }
  return false;
}

void MideaXYE::send_c3_short_() {
  // 16-byte C3 wake-up. Carries the controller's desired wire mode/fan/setpoint
  // — exactly the same encoding as a C4-set's mode byte. Captured controller
  // sent 0x08 (COOL no power) when commanding the AC OFF; 0x88 (COOL + power)
  // when commanding ON.
  uint8_t f[16] = {
    0xAA, 0xC3, 0x00, 0x00, 0x00, 0x00,
    this->wire_mode_byte_(),  // [6] mode (power | remembered bits)
    this->desired_fan_,       // [7]
    this->desired_setpoint_,  // [8]
    0x00, 0x00, 0x00, 0x00, 0x3C, 0x00, 0x55,
  };
  f[14] = calc_checksum_(f, 16);
  this->write_array(f, 16);
  ESP_LOGI(TAG, "TX C3-short (cmd: mode=0x%02X fan=0x%02X set=0x%02X)",
           f[6], f[7], f[8]);
}

void MideaXYE::send_c3_long_() {
  // 32-byte C3 wake-up. Captured behavior: this is a COPY of the AC's last
  // 0xC0 status frame with byte[1] swapped from 0xC0 → 0xC3 and byte[8]
  // forced to 0x00. If we haven't yet received a C0 to mirror, fall back to
  // a zeroed-payload template (most fields are 0 in the real frame anyway).
  uint8_t f[32];
  if (this->have_c0_status_) {
    memcpy(f, this->last_c0_status_, 32);
    f[1] = 0xC3;
    f[8] = 0x00;
  } else {
    static const uint8_t TEMPLATE[32] = {
      0xAA, 0xC3, 0x00, 0x00, 0x00, 0x00, 0x30, 0x14,
      0x00,        // [8] mode forced to 0x00
      0x81,        // [9] fan AUTO+HIGH (typical idle value)
      0x51,        // [10] setpoint placeholder
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x55,
    };
    memcpy(f, TEMPLATE, 32);
  }
  // Always recompute checksum since we mutated bytes 1 + 8.
  f[30] = calc_checksum_(f, 32);
  this->write_array(f, 32);
  ESP_LOGI(TAG, "TX C3-long (status-echo, %s)",
           this->have_c0_status_ ? "real C0 mirrored" : "blank template");
}

void MideaXYE::send_c6_short_() {
  // 16-byte C6 identify. All constants.
  static const uint8_t F[16] = {
    0xAA, 0xC6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x06, 0x13, 0x00, 0x39, 0xE8, 0x55,
  };
  this->write_array(F, 16);
  ESP_LOGI(TAG, "TX C6-short (identify)");
}

void MideaXYE::send_c6_long_() {
  // 32-byte C6 identify with state. Mirrors C4-set layout (mode/fan/setpoint
  // at the same offsets). Verified against captured controller: byte[16] tracks
  // the same value the controller is sending in its C4-set (e.g. 0x08 when off,
  // 0x88 when on).
  uint8_t f[32] = {
    0xAA, 0xC6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x30, 0x18, 0x00, 0x00, 0x00, 0x00,
    0x20,                     // [15] flags (no turbo during boot)
    this->wire_mode_byte_(),  // [16]
    this->desired_fan_,       // [17]
    this->desired_setpoint_,  // [18]
    0xBE, 0xD6, 0x61, 0x00, 0x00, 0x20, 0x00, 0x80, 0x80, 0x80, 0x00,
    0x00, 0x55,
  };
  f[30] = calc_checksum_(f, 32);
  this->write_array(f, 32);
  ESP_LOGI(TAG, "TX C6-long (mode=0x%02X fan=0x%02X set=0x%02X)",
           f[16], f[17], f[18]);
}

void MideaXYE::send_c4_set_() {
  // Template copied verbatim from a verified-good capture.
  uint8_t frame[32] = {
    0xAA, 0xC4, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x30, 0x18, 0x00, 0x00, 0x00, 0x00,
    0x20,        // [15] flags (bit 3 = turbo)
    0x00,        // [16] mode  (overwritten below)
    0x00,        // [17] fan   (overwritten below)
    0x00,        // [18] setpoint (overwritten below)
    0xBE, 0xD6, 0x64, 0x00, 0x00, 0x20, 0x00, 0x80, 0x80, 0x80, 0x00,
    0x00,        // [30] checksum (computed below)
    0x55,
  };

  frame[15] = 0x20 | (this->desired_turbo_ ? 0x08 : 0x00);
  frame[16] = this->wire_mode_byte_();
  frame[17] = this->desired_fan_;
  frame[18] = this->desired_setpoint_;
  frame[21] = 0x61;
  frame[30] = calc_checksum_(frame, 32);

  this->write_array(frame, sizeof(frame));
  ESP_LOGI(TAG, "TX C4-set mode=0x%02X fan=0x%02X set=0x%02X(%dC)",
           frame[16], frame[17], frame[18], frame[18] - 0x40);
}

void MideaXYE::publish_status_(const uint8_t *data) {
  // Stash this for our C3-long boot frame, which the wired controller composes
  // by mirroring the AC's most recent C0 status (with byte[1] → 0xC3, byte[8] → 0x00).
  memcpy(this->last_c0_status_, data, LONG_LEN);
  this->have_c0_status_ = true;

  if (this->t1_sensor_) this->t1_sensor_->publish_state(decode_temp_c_(data[11]));
  if (this->t2_sensor_) this->t2_sensor_->publish_state(decode_temp_c_(data[12]));
  if (this->t3_sensor_) this->t3_sensor_->publish_state(decode_temp_c_(data[14]));
  if (this->fan_speed_text_sensor_) {
    // Lower nibble of byte[9] holds the actual current spinning speed, set by
    // the AC regardless of whether AUTO is requested (bit 7). 0x00 = idle.
    const char *speed;
    switch (data[9] & 0x0F) {
      case 0x01: speed = "high"; break;
      case 0x02: speed = "medium"; break;
      case 0x04: speed = "low"; break;
      case 0x00: speed = "off"; break;
      default:   speed = "unknown"; break;
    }
    this->fan_speed_text_sensor_->publish_state(speed);
  }
  if (this->climate_) this->climate_->on_status_frame(data);
}

}  // namespace midea_xye
}  // namespace esphome
