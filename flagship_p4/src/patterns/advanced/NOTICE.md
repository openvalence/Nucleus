# Notice: the Advanced pattern unit

This directory is licensed under the CERN Open Hardware License version 2,
Strongly Reciprocal (CERN-OHL-S-2.0), full text in `LICENSE` here. The rest of
Nucleus is Apache-2.0 (the repository root's `LICENSE` and `NOTICE.md`).

## Origin

Derived from OSSM-Lite by fray-d, https://github.com/fray-d/OSSM-Lite,
CERN-OHL-S-2.0: the Advanced Penetration modifier and half-stroke math.

## Modifications (CERN-OHL-S-2.0 section 3.3(b))

Modified by AtlanticTM, 2026-09-25 through 2026-10-03:

- Float math throughout where the original used u8 arithmetic.
- Linear knobs: speed is master percent times half percent times the input
  ceiling, the accel knob is the accel percent; the original eased both.
- Names follow Valence RFC-066 (modulators) and RFC-095 (crest and trough
  dwells).
- The half-stroke duration rule (`halfStrokeSeconds`) is a named function the
  pattern engine calls, so no fray-d expression lives outside this directory.

## Source Location (section 3.3(c))

https://github.com/openvalence/Nucleus, path `flagship_p4/src/patterns/advanced/`.

## What stays outside this directory

The Advanced modulator field definitions in
`flagship_p4/src/hub/ValenceCatalog.h` (names, ranges, RFC-066) are interface
vocabulary ratified in the Valence spec, not fray-d expression; they carry a
fray-d credit comment and are Apache-2.0. The preset store kind name
`pattern.frayd` (`PatternPresetStore.h`, `ValenceCatalog.h`) is an identifier.
