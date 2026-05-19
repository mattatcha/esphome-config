# Phase 3 — current blocker & next experiment

## Where we left off

Last commit (`133a988`) added the missing **C0-poll** frame to both the boot
sequence and the regular cycle. Previously we were sending C4-poll + C4-set
and waiting for the AC to respond — but the AC doesn't respond to those. It
responds to a **C0-poll** (16 bytes, `AA C0 00 00 00 00 00 00 00 00 00 00 00 3F 01 55`)
with a `0xC0` 32-byte status frame.

Without C0-poll in our cycle, we never gave the AC the chance to send its
status. That's why no `AA C0 …` 32-byte frames ever arrived in earlier Phase 3
tests.

## Next experiment (flash + observe)

1. User flashes the current build to `master-minisplit.local`.
2. User attaches to logs (`esphome logs carrier-minisplit-v4/carrier-minisplit.yaml`).
3. User presses the **`XYE Start TX`** button in HA.
4. Watch the log for the next 5-10 seconds.

### Success criteria

A `[RX C0-status (AC) len=32]` line should appear within ~150 ms of every
`TX C0-poll`. The status frame should decode into real temperatures (T1, T2,
T3 in plausible ranges around room/coil/outdoor temps).

Expected steady-state cycle in the log:

```
[I] TX C4-poll
[I] TX C4-set mode=0x08 fan=0x80 set=0x56(22C)
[I] TX C0-poll (request status)
[I] [RX C0-status (AC) len=32] AA C0 00 00 00 00 30 14 …  ← THIS is the win
[I]   status: power=… mode=… fan=… set=… T1=… T2=… T3=…
```

### Failure modes & what they mean

- **No `RX C0-status (AC)` after `TX C0-poll`.** Our C0-poll bytes might
  differ from the captured controller's. Do a logic capture during a TX
  cycle, compare our C0-poll byte-for-byte against the captured one in
  `CAPTURES.md` Session 8 (`AA C0 00 00 00 00 00 00 00 00 00 00 00 3F 01 55`).
- **`RX C0-status (AC)` appears but content is mostly zeros / `0xFF` sentinel
  values.** The AC is replying but considers us not-fully-registered. Compare
  against captured C0-status content — there may be a master-side byte
  signature the AC checks before populating real data.
- **Bus collision (frames overlap, validation fails).** Our line-quiet gate is
  60 ms — possibly too short for this AC's response latency. Bump
  `TX_LINE_QUIET_MS` higher or add a longer post-C4-set gap.

## If C0-status comes through cleanly

Then Phase 3 RX side is done. Next sub-task: confirm the **state-change
commit** flow works.

1. With AC engaged (steady C0-status incoming), change the HA climate's mode
   from OFF to COOL.
2. Logs should show:
   - `[I] desired state: power=ON mode_bits=0x08 (wire=0x88) … [will commit via C3-short next cycle]`
   - `[I] TX C3-short (cmd: mode=0x88 …)`
   - `[I] [RX C0-status (AC) …]` with `mode=0x81` (FAN_ONLY, transitional)
   - A second later: `[I] [RX C0-status (AC) …]` with `mode=0x88` (COOL, fully on)
3. AC fan should physically turn on. Compressor engages a few seconds later.

If only the C3-short fires but the AC doesn't transition, we likely need the
**full C3-short → C3-long → C6-short → C6-long sub-sequence** on every state
change, not just C3-short. The captured controller does the full sequence
when the user presses power-on (`CAPTURES.md` Session 8, t=+7155 ms onward).

Implementation hook: extend the `state_dirty_` handler in `tx_pump_()` to walk
through 4 commit phases instead of just sending one C3-short. There's already
a working pattern (the boot phase state machine) to copy from.

## If C0-status still doesn't come through

Most likely culprits, ranked:

1. **Our C0-poll bytes are subtly wrong.** Capture our actual TX with the
   logic analyzer during a cycle, diff against captured controller bytes.
2. **Line-quiet timing too aggressive.** Try `TX_LINE_QUIET_MS = 100` or
   higher. The AC might not get a clean window to start its response if we
   re-TX too quickly.
3. **`byte[7]` in our master TX is wrong.** We send `0x00`; the controller
   also sends `0x00`. But maybe in some frames the AC expects its address
   (`0x14`) echoed? Worth checking each frame's byte[7] in the captured
   reference.
4. **TX/RX transceiver auto-direction switching.** If the transceiver doesn't
   release the bus fast enough after our TX, the AC's response window may be
   stomped on. Hardware-side, harder to fix.

## Diagnostic shortcut

The `XYE One-Shot Set 28C` button fires a single C4-set frame with setpoint
28 °C. Useful for "send one frame, see what comes back over the next 500 ms"
testing without the periodic cycle adding noise. Pair with `XYE Stop TX` to
get a quiet bus for diagnostic frames.
