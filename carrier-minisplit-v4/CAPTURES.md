# Carrier/Midea XYE Bus — Capture Log & Protocol Notes

Working notes from reverse-engineering the RS-485 bus between a Carrier mini-split
indoor unit and its wired wall controller (rebadged Midea XYE protocol). All
captures done with a Saleae Logic device probing the bus single-ended.

## Frame structure

All frames share the same outer envelope:

```
[0]      0xAA            start byte
[1]      type            see frame-type table below
[len-2]  checksum        0xFF - (sum_of_all_bytes_except_self & 0xFF)
                         sum includes start byte AND trailer
[len-1]  0x55            end byte
```

Two valid lengths: **16 bytes** (short) and **32 bytes** (long).

### Frame types

| Type | Length | Source | Purpose |
|------|--------|--------|---------|
| `0xC4` | 16 | Master | Heartbeat poll. Constant content `AA C4 00*11 3B 01 55`. |
| `0xC4` | 32 | Master | State push ("I want this state"). Carries mode/fan/setpoint. |
| `0xC0` | 16 | Master | Status request to slave. Constant `AA C0 00*11 3F 01 55`. |
| `0xC0` | 32 | **AC (slave)** | Status response. Carries real sensor readings + state echo. |
| `0xC3` | 16 | Master | "Wake / commit" command. Carries mode/fan/setpoint to enact a state change. |
| `0xC3` | 32 | Master | "Status mirror" — copy of AC's last C0-status with byte[1] swapped to 0xC3 and byte[8] forced to 0x00. |
| `0xC6` | 16 | Master | Identify / capability beacon. Mostly constants. |
| `0xC6` | 32 | Master | Capability echo. Same layout as C4-set. |
| `0xC5` | varies | AC | Autonomous "no master detected" beacon. Sent when no master is on the bus. |
| `0x00` | 1 byte | Master | Bus-break preamble. Master sends one `0x00` byte ~1.7 s before its first real frame on startup. |

### Cycle (steady state, with master present)

```
Master  → C4-poll  (16 B, heartbeat)
Master  → C4-set   (32 B, state push)
Master  → C0-poll  (16 B, "give me status")
   AC   → C0-status (32 B, actual data)            ← AC replies ~75 ms after C0-poll
```

Repeats roughly every 600 ms – 1 s. Observed: ~75 ms between C0-poll and AC's
C0-status; ~80 ms between successive master frames in a single cycle.

### Startup choreography

When the wired controller boots up (or reconnects), it sends a specific 8-step
sequence before resuming the regular cycle:

```
BREAK (single 0x00 byte)
  ↓ ~1.7 s
C4-poll
  ↓ ~80 ms
C4-set                    ← carries controller's last-remembered state
  ↓ ~70 ms
C0-poll
  ↓ ~80 ms (AC replies with C0-status here)
C3-short                  ← commits user's desired state (may differ from C4-set's)
  ↓ ~80 ms
C3-long                   ← mirrors AC's last C0-status with byte[8] = 0x00
  ↓ ~70 ms
C6-short
  ↓ ~90 ms
C6-long                   ← carries user's desired state (same as final C3-short content)
  ↓
Regular cycle resumes
```

When the user changes anything mid-stream (e.g. presses power-on), the controller
fires the same C3-short → C3-long → C6-short → C6-long sub-sequence to commit the
change, then resumes the C4 cycle with the new state.

## Field encodings (32-byte C0 status)

All offsets are in the wire frame (including the `AA C0` header).

| byte | meaning | encoding |
|------|---------|----------|
| 6 | constant `0x30` | header — semantics unknown |
| 7 | AC address / unit ID | constant `0x14` on this unit |
| 8 | mode | bit 7 = power; lower bits: `0x00` AUTO / `0x08` COOL / `0x04` HEAT / `0x02` DRY / `0x01` FAN_ONLY. Full byte `0x00` = OFF |
| 9 | fan | bit 7 = AUTO overlay; lower bits: `0x01` HIGH / `0x02` MED / `0x04` LOW. e.g. `0x84` = AUTO currently running LOW |
| 10 | setpoint | °C mode: `raw - 0x40 = °C`. °F mode: high bit set, `raw & 0x7F = °F` |
| 11 | T1 — indoor return air | `(raw - 40) / 2 °C` |
| 12 | T2 — indoor coil | `(raw - 40) / 2 °C` |
| 14 | T3 — outdoor coil | `(raw - 40) / 2 °C` |
| 19 | flag bits | bit 4 (`0x10`) seen during normal op (meaning TBD — *not* turbo as initially assumed); bits 0/1 toggle when compressor cycles |
| 30 | checksum | per outer-frame rule |

### 32-byte C4 set (master TX)

Different layout — mode/fan/setpoint live at higher offsets.

| byte | meaning |
|------|---------|
| 9, 10 | constants `0x30 0x18` |
| 15 | flags. `0x20` baseline. Bit 3 (`0x08`) = turbo requested. |
| 16 | mode (same encoding as C0[8]) |
| 17 | fan |
| 18 | setpoint |
| 19, 20 | constants `0xBE 0xD6` (semantics unknown) |
| 21 | small drift `0x60..0x65` (possibly sequence counter / activity indicator) |
| 24 | constant `0x20` |
| 26-28 | constants `0x80 0x80 0x80` |

### 16-byte C3 short (master "commit" command)

| byte | meaning |
|------|---------|
| 6 | mode (full wire byte, e.g. `0x88` to turn on COOL) |
| 7 | fan |
| 8 | setpoint |
| 13 | constant `0x3C` |

### T4 (outdoor ambient air)

The wall controller's service menu shows a T4 reading. **T4 is not present in
any byte of the C0 status frame.** Either it's never reported on this bus, or
it's only sent in response to a service-menu query frame we haven't captured.

### Follow-me & wall-sensor display

The wall controller's local sensor reading **never goes on the bus**. Heating
the wall controller with a hair dryer (up to 50 °C displayed) produced zero
changes in any frame the controller sent. "Follow-me" mode is purely a
controller-internal behavior: the controller uses its sensor locally to decide
when to issue commands; it never tells the AC what the wall sensor reads.

## Capture log

### Session 1 — controller-only, no AC

`/tmp/xye-d0-poweron.csv` (Logic capture #9), 4.21–13.4 s window.

Probe: D0, single-ended on one conductor of the differential pair. Inverted
decode required (because of A/B orientation at the time).

Result: clean repeating `AA C4 00 00 00 00 00 00 00 00 00 00 00 3B 01 55` every
240 ms. No replies — there was no AC to respond. Established the C4-poll frame
shape and the bus's idle behavior.

### Session 2 — setpoint sweep (17 → 30 → 17 °C)

Logic capture #12. Setpoint changed on controller dial while bus was being
recorded.

Findings:
- `byte[18]` of C4-set and `byte[10]` of C0-status both moved in lockstep with
  the displayed setpoint.
- Encoding: `wire_byte = °C + 0x40`. e.g. 17 °C → `0x51`, 30 °C → `0x5E`.
- Range used: `0x51` to `0x5E`.

### Session 3 — fan speed cycling

Logic capture #14. Cycled AUTO → HIGH → MED → LOW on the controller.

Findings:
- HIGH = `0x01`, MED = `0x02`, LOW = `0x04`, AUTO = `0x80` (intent byte from
  master).
- AC's response in C0 status: bit 7 = AUTO active, lower nibble = *current*
  spinning speed. So AUTO+HIGH = `0x81`, AUTO+LOW = `0x84`, etc.

### Session 4 — hair-dryer on wall sensor

Logic capture #15. Heated wall controller with a hair dryer; display went from
24 °C → 50 °C → 26 °C.

Findings:
- **Zero on-wire changes that match the wall display.** The wall controller's
  local sensor is not transmitted to the AC.
- `byte[7]` of C0 was `0x14` constantly — confirmed this is an address/unit-id,
  not a temperature.

### Session 5 — hair-dryer on AC intake (T1)

Logic capture #17. T1 started at 17 °C, peaked at ~50 °C, ended at 38 °C.

Findings:
- `byte[11]` of C0 status moved from `0x4B` to `0x76` and back to `0x73`.
- Formula `(raw - 40) / 2 °C`: `0x4B` → 17.5 °C, `0x76` → 39 °C, `0x73` → 37.5 °C.
  Matches displayed values within 0.5 °C.
- This corrects the v2 library's `(raw - 50) / 2` formula by 5 units in the
  offset constant.

### Session 6 — heat T1 with follow-me OFF

Logic capture #18. Same hair-dryer test, but with follow-me disabled so the AC
uses its own T1 sensor for thermostat decisions. User reported "everything
ramped up."

Findings:
- `byte[12]` (T2 coil) dropped to 3 °C (evaporator running hard).
- `byte[14]` (T3 outdoor coil) rose to 35.5 °C under compressor load.
- Confirmed T3 is the outdoor *coil* (heats up under load), not outdoor *air*.
  T4 (ambient air) is still missing from the on-wire data.

### Session 7 — turbo mode toggle

Logic capture #20. Toggled "turbo air" mode on the controller off → on → off.

Findings (at the time):
- C4-set byte[15] flipped `0x20 → 0x28` (bit 3 set) when turbo requested.
- C0-status byte[19] flipped `0x03 → 0x13` (bit 4 set) when turbo active.

Caveat: a later capture showed `byte[19] bit 4` set continuously while idle, so
that earlier "bit 4 = turbo" reading is suspect. The flag-byte semantics need a
cleaner re-test.

### Session 8 — wired-controller cold-boot reference

ESP-side log at 22:18:36–22:18:50 in `carrier-minisplit-v4` Phase 1 sniffer
mode. Controller had been disconnected; reconnected and observed booting up.
Then user pressed power-on at 22:18:43.

This is **the** ground-truth capture for protocol behavior. Key transitions
extracted from it (relative offsets from the BREAK byte):

```
   +0 ms      BREAK (0x00)
+1720 ms      C4-poll
+1814 ms      C4-set     mode=0x08 (COOL/off)  set=0x56  byte21=0x5F
+1883 ms      C0-poll
+1964 ms      C0-status (AC)  mode=0x00  fan=0x84  set=0x56  T1=22.5 T2=21.5 T3=27
+2027 ms      C3-short   mode=0x08      set=0x51  ← commits new setpoint 17°C
+2108 ms      C3-long    (mirrors C0-status, byte[8] forced to 0x00)
+2180 ms      C6-short
+2271 ms      C6-long    mode=0x08  set=0x51
+2496 ms      C4-poll    (regular cycle resumes with new state)
   ...
+7155 ms      C3-short   mode=0x88  ← user pressed power-on
+7237 ms      C3-long    transitional (mode=0x81 FAN_ONLY)
+7307 ms      C6-short
+7395 ms      C6-long    mode=0x88
+7605 ms      C4-poll    (cycle resumes, now with mode=0x88)
+8609 ms      C0-status reports mode=0x88 COOL ON, flags transition 0x10 → 0x13
```

Critical findings from this capture:
1. The 16-byte `AA C0 … 3F 01 55` frame the controller sends ~70 ms after C4-set
   was previously mislabeled as an "ack" from the AC. It is actually a *poll*
   from the controller asking the AC for its status.
2. State changes (initial boot setpoint difference AND user power-on press) both
   trigger the same C3-short → C3-long → C6-short → C6-long sub-sequence. This
   is the "commit" choreography.
3. AC transitions through `mode=0x81` (FAN_ONLY, transitional) for ~1 s before
   reaching the final mode (e.g. 0x88 COOL) — the fan spins up before the
   compressor engages.
4. Setpoint encoding differs between captures. This one used °C (+0x40); an
   earlier capture had the controller in °F mode, sending `0xC5` (= 0x80 | 69)
   for "69 °F". Our parser now handles both.

## Open questions

- **Why the AC isn't engaging as slave when we're the master.** Even after
  matching the captured controller's byte content and timing very closely, the
  AC's C0 status responses haven't appeared in Phase 3 tests. The C0-poll
  addition (Session 8 finding) is the most recent change; needs flash + retest.
- **What `byte[19]` flags actually mean.** Bit 4 was set during idle in
  Session 8, contradicting the earlier "bit 4 = turbo" reading from Session 7.
  Bits 0/1 transitioning when compressor cycles is the only solid observation.
- **`byte[21]` of C4-set drift** (`0x5F`..`0x65`). Possibly a sequence counter,
  activity beat, or some opaque per-cycle value. The AC sees our hardcoded
  `0x61` and doesn't object, but worth understanding.
- **T4 source.** Service-menu shows it; bus doesn't carry it (in any frame
  we've captured). Possibly retrievable via a different request type, or
  computed locally by the controller.
- **`AA C5` semantics.** AC emits these when no master is present. Layout
  has 8 bytes of `0xFF` in the middle that look like "uninitialized sentinels."
  Not relevant once we're successfully master.

## Things we tried that didn't work

Recorded so future sessions don't retry the same dead ends.

- **Hardcoded `mode=0x88` (COOL ON) in boot frames as a "wake-up" signal.**
  Hypothesis was that the AC wouldn't engage as slave unless commanded ON.
  Disproved by Session 8: the wired controller booted with `mode=0x08` (off)
  and the AC happily replied with C0-status anyway. Mode bit-7 is not what
  triggers slave engagement.
- **Pulling the upstream `esphome-mideaXYE-rs485` package via the GitHub
  `packages:` URL.** The GitHub HEAD is older than the user's local
  uncommitted edits; pulling from the URL gave us a stripped-down version
  missing the bootstrap interval and `hasValidState` gating. Use a local
  `!include` instead (see v3 yaml). v4 doesn't depend on it at all.
- **Sending only a single `C3-short` to commit state changes after boot.**
  Insufficient — the captured controller sends the full `C3-short → C3-long →
  C6-short → C6-long` sub-sequence on every user-initiated state change,
  not just `C3-short`. Current v4 build still does only C3-short; needs
  upgrading once C0-status RX is confirmed working.
- **Treating the 16-byte `AA C0` frame as the AC's ack.** It's actually a
  master-direction frame: the controller's "give me status" poll. The AC's
  only on-wire output is the 32-byte `AA C0` status frame.
- **Hypothesizing `byte[19] bit 4 = turbo`** based on a single toggle
  observation in Session 7. Session 8 showed bit 4 set continuously while
  the AC was idle. Don't trust that decode without re-verification.
- **Assuming the wall controller's sensor reading transmits on the bus.**
  Session 4 (hair-dryer on wall) produced zero on-wire changes. Wall sensor
  is local-only; "follow-me" is a controller-internal behavior.
- **Sending `mode = 0x00` (OFF with no remembered bits) when HA says OFF.**
  The wired controller never sends `mode=0x00` on the wire — even when OFF,
  it carries the remembered mode bits (e.g. `0x08` = "COOL but powered off").
  The AC may treat `0x00` as malformed. Always send `remembered_bits |
  (power ? 0x80 : 0)`.

## Files

- `components/midea_xye/midea_xye.h/.cpp` — frame parser, TX state machines,
  field decoders.
- `components/midea_xye/midea_xye_climate.{h,cpp}` — climate platform entity.
- `components/midea_xye/{climate,sensor,text_sensor}.py` — platform schemas.
- `carrier-minisplit.yaml` — device config, including diagnostic buttons.
- `CLAUDE.md` — project-local Claude session instructions.
- `PHASE-3-TODO.md` — current blocker and next concrete experiment.
