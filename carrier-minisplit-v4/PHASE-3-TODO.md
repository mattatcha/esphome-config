# Status (2026-05-19)

**Phase 3 (RX side / AC engagement) is DONE.** Session 9 in `CAPTURES.md` has
the full discovery. TL;DR: a 16-byte `AA C0 00*11 3F 01 55` poll every 1 s is
sufficient to elicit a 32-byte C0 status from the AC within ~150 ms. The
elaborate boot handshake replicated from the wired controller is not needed.

`use_fahrenheit: true` YAML option also added — necessary because this AC's
display is set to °F, which changes how byte 10 (setpoint) is encoded on the
wire.

# Phase 4 — state commit (current WIP)

Goal: send mode/fan/setpoint commands from HA → AC and have the AC reflect
the new state in its subsequent C0 status responses.

## Next experiment (flash + observe)

The plan, in the spirit of "smallest possible test":

1. User has `XYE Minimal C0 Poll` running (steady 1 s C0 cycle, AC responding).
2. User presses a new `XYE Minimal C3 Set` button that fires ONE 16-byte C3
   frame with: `mode=COOL+power (0x88)`, `fan=AUTO (0x80)`, `setpoint=72 °F
   (0xC8)` — picks values different from the current AC state so a change is
   unambiguous.
3. Watch the next ~3 C0 status frames in the log.

### Success criteria

The C0 status frame following the C3 set should show byte 10 transition from
the prior setpoint to `0xC8` (or `0x48`, see "Setpoint TX encoding" below).
Mode byte 8 should reflect `0x88` (if it wasn't already). Fan byte 9 may
transition through `0x80` (AUTO requested, fan still idle) then show speed
when fan engages.

### Failure modes & what they mean

- **No state change in C0 status, AC keeps reporting old values.** Either:
  - Our single 16-byte C3 isn't enough — the AC needs the full `C3-short →
    C3-long → C6-short → C6-long` choreography. Next step: replicate that
    sub-sequence but still as 16-byte frames (no 32-byte master TX).
  - Or our C3 frame bytes are subtly malformed. Diff against a logic capture
    of the wired controller's C3 (`CAPTURES.md` Session 8 has a known-good
    frame at t=+2027 ms).
- **Bus collision (frames overlap, validation fails).** Bump `TX_LINE_QUIET_MS`
  or add an explicit "wait for line quiet" before the one-shot TX.
- **AC responds with an `AA C0` short frame complaining (e.g. CCM error
  flag byte 26 != 0).** Inspect that flag — 0x02 = CRC error, 0x04 = protocol
  error. Tells us if we're getting noticed but rejected.

### Setpoint TX encoding — the unsettled bit

Two reasonable forms for 72 °F in TX:
- `0xC8` = bit 7 set + 72 (matches the captured wired controller's °F TX)
- `0x48` = plain decimal 72 (mirrors the AC's RX-side encoding)

The button starts with `0xC8` since that matches a confirmed-good capture.
If the AC doesn't respond, try `0x48` next.

## After SET works

If C3 SET commits state cleanly:
- Plumb the climate `control()` call into a real SET path (replace the
  current 32-byte C4-set + boot handshake plumbing entirely).
- Adopt HomeOps's full cycle: QUERY (C0) → QUERY_EXTENDED (C4) → QUERY → …,
  with SET (C3) and FOLLOW_ME (C6) interleaved on user changes.
- Decode the 32-byte C4 response (now strongly suspected to carry outdoor
  temp + static pressure + compressor flags etc., per HomeOps).
- Rip out the 8-phase boot machinery — it's confirmed unnecessary.

If C3 SET does *not* commit state on its own:
- Add a "XYE Full C3+C6 Set" button that fires C3-short → C3-long → C6-short
  → C6-long as 16-byte frames in sequence (no 32-byte master TX).
- If that works, the choreography matters but the byte count doesn't.
- If that still doesn't work, we need to compare logic captures of our TX
  against the wired controller's TX during a known-good user state change.
