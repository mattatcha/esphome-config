# carrier-minisplit-v4 — project notes

ESPHome integration for a Carrier mini-split using the Midea XYE protocol
over RS-485. Custom external component at `components/midea_xye/`.

## Status

- **Phase 1 (RX sniffer)** — shipped and working. Frame parser handles 16- and
  32-byte `AA…55` frames with simple-sum checksum, plus loose-close fallback
  for partial frames recovered from inter-byte timeout.
- **Phase 2 (HA entities)** — shipped. Climate + per-temperature sensors +
  fan-speed text sensor populate from the AC's `0xC0` status frames.
- **Phase 3 (TX path / master replacement)** — work in progress. The AC isn't
  yet engaging as a slave when we transmit. See `CAPTURES.md` § Open questions
  and `PHASE-3-TODO.md` for the current blocker and the next concrete
  experiment to try.

## Workflow rules

- **You can't flash from this machine.** Write code, validate it, surface what
  the user should test, then wait for them to flash and paste logs. Don't
  pretend a code change is working until the user confirms.
- **Validate every change** with `mcp__esphome__validate_config` (and
  `compile_config` for C++ changes) before claiming the change is good. They
  catch yaml/code errors instantly.
- **Use `logic2` MCP tools for bus captures.** The user has a Saleae Logic
  hooked up to the RS-485 bus. Start captures, decode with the Async Serial
  analyzer (4800 baud 8N1, non-inverted), export CSV to `/tmp/`.
- **Default to diff-based protocol RE.** Toggle one variable on the controller
  (or in the HA UI), capture before+after, find the byte that moved. Don't
  hypothesize byte semantics without a capture-driven test to confirm.
- **Don't mutate boot/handshake byte content speculatively.** The current
  values are verified against a captured wired-controller cold-boot
  (`CAPTURES.md` Session 8). If you must change them, do a new comparison
  capture first.

## Where things are

- **Device**: `master-minisplit.local` (192.168.2.214)
- **Component**: `components/midea_xye/` (`midea_xye.{h,cpp}` core,
  `midea_xye_climate.{h,cpp}` entity, `.py` files for platform schemas)
- **Device config**: `carrier-minisplit.yaml`
- **Logic captures**: ad-hoc, exported to `/tmp/*.csv` during sessions
- **Reference data**: `CAPTURES.md` (chronological capture log + verified
  protocol decode); `PHASE-3-TODO.md` (next steps)

## Common pitfalls

- The 16-byte `AA C0 …` frame is **NOT** the AC's ack — it's the master's
  status-request poll. Mislabeled as "ack" through most of v4's development;
  fixed now but easy to backslide. Only the **32-byte** `AA C0 …` comes from
  the AC.
- Frames received during our own TX may be our transceiver echoing TX→RX
  (loopback). When we're the master, every `AA C4 / AA C3 / AA C6 / 16-byte AA C0`
  in RX is almost certainly our own TX, not the AC.
- Setpoint encoding has **two modes**: °C (`raw - 0x40`) when high bit clear,
  and °F (`raw & 0x7F`) when high bit set. Controller switches based on its
  display unit preference.
- Temperature formula `(raw - 40) / 2` °C — the upstream xyeVars.h library uses
  `(raw - 50) / 2` which is off by 5 in the offset.
- `byte[19]` flag semantics in C0 status are still partially unknown. Earlier
  reading "bit 4 = turbo" is suspect — bit 4 has been seen set continuously
  while idle. Don't rely on flag-byte decode without re-verifying.

## Useful runtime buttons (in HA after flash)

Current preferred path (Phase 3+):
- **`XYE Minimal C0 Poll`** — sends only `AA C0 00*11 3F 01 55` every 1 s, no
  boot handshake, no other frames. The AC engages on this alone (Session 9
  in CAPTURES.md). Default `tx_enabled: false`, so this button is the
  intended way to start TX.
- **`XYE Minimal C3 Set`** — Phase-4 one-shot SET test (mode=COOL ON, fan=AUTO,
  setpoint=72 °F). Fires while minimal-poll is running.
- **`XYE Stop TX`** — halts both minimal-poll and any boot-handshake TX
  (RX parser keeps running).

Legacy (kept for reference / comparison):
- **`XYE Start TX`** — kick off the 8-phase boot handshake and start the
  periodic C4 cycle. Confirmed unnecessary by Session 9 but useful for
  comparing wired-controller-style chatter against the minimal path.
- **`XYE One-Shot Set 28C`** — fires a single 32-byte C4-set frame with
  setpoint 28 °C (°C-mode encoding). Diagnostic only — the AC doesn't
  necessarily react to this; the new C3-set button is the right tool for
  Phase-4 testing.
