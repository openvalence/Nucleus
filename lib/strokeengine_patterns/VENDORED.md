# Vendored: StrokeEngine pattern classes

Upstream:   https://github.com/theelims/StrokeEngine (pattern math), as carried by
            https://github.com/KinkyMakers/OSSM-hardware, branch `main`,
            path `Software/lib/StrokeEngine/src/`
License:    MIT, Copyright (C) 2021 theelims. LICENSE is the standard MIT
            text with the copyright line from pattern.h's own header; upstream's
            LICENSE file was not fetched.

## What is included

- `include/pattern.h` -- the `Pattern` base class and the seven core patterns
  (SimpleStroke, TeasingPounding, RoboStroke, HalfnHalf, Deeper, StopNGo,
  Insist).
- `include/PatternMath.h` -- `fscale`, `fmap`, `mapSensationToFactor`.
- `include/patternShim.h` -- a first-party shim (`motionParameter`,
  `STRING_LEN`). Not upstream, but `pattern.h` includes it by name, so it
  lives beside it.

`patternExtended.h` (the community test patterns) is deliberately NOT vendored:
the catalog advertises only the seven core names.

## Local modifications

`pattern.h` differs from upstream by one comment word respelled to American
English for C-11 (line 66). `patternShim.h` is first-party and edited here.
Upstream byte-identity of `pattern.h` was not re-verified from this side.

## How it builds without Arduino

`pattern.h` includes `<Arduino.h>` and calls `millis()`, `Serial.println()`,
`String`, `map()` and `constrain()`. Nucleus is pure ESP-IDF, so those names
come from a first-party adapter,
`flagship_p4/src/patterns/arduino_compat/Arduino.h`, which every consumer puts
on its include path. It is hardware-free, allocates nothing (`String` and
`Serial` are no-ops, so the `DEBUG_PATTERN` prints compile away), and its
`millis()` reads a clock the pattern engine sets from its injected time. Wrap,
never patch: behavior this project needs lives in
`flagship_p4/src/patterns/PatternEngine.*`.

## Update procedure

1. Re-copy `pattern.h` and `PatternMath.h` from upstream.
2. Record any delta above.
3. `pio test -e native` (suite `test_pattern_engine`), the sim build and
   `pio run -d flagship_p4` must all stay green before commit.
