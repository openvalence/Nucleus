#pragma once

// Arduino -- the handful of Arduino names the vendored StrokeEngine patterns
// call, on a target with no Arduino core
// Constraints:
// - HARDWARE-FREE and ALLOCATION-FREE. String and Serial are inert stand-ins:
//   pattern.h hard-defines DEBUG_PATTERN, so its debug prints must compile, and
//   here they compile to nothing. Never grow String into a real string: the
//   patterns call those prints on every parameter change, which is steady
//   state, where heap allocation is banned (cpp-safety.md).
// - millis() is NOT a clock. It returns arduino_compat::g_now_ms, which the
//   pattern engine sets from its injected time before every call into a
//   pattern. One engine per task: the value is process-global.
// - Included only by lib/strokeengine_patterns and the TUs that include it
//   (the pattern engine, its hosts, its test). Nothing else may include it.
// See: lib/strokeengine_patterns/VENDORED.md, patterns/PatternEngine.h

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <math.h>

namespace arduino_compat {
inline uint32_t g_now_ms = 0;
}  // namespace arduino_compat

inline unsigned long millis() { return arduino_compat::g_now_ms; }

// Arduino's integer map(): pattern.h hands it a float, which converts to long
// exactly as it does on the Arduino core.
inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

template <class T, class L, class H>
inline T constrain(T amt, L low, H high) {
    return amt < T(low) ? T(low) : (amt > T(high) ? T(high) : amt);
}

class String {
public:
    String() = default;
    template <class T>
    explicit String(const T&) {}
    template <class T>
    String(const T&, int) {}
};
inline String operator+(const char*, const String&) { return {}; }
inline String operator+(const String&, const char*) { return {}; }
inline String operator+(const String&, const String&) { return {}; }

struct ArduinoSerialSink {
    template <class T>
    void println(const T&) {}
};
inline ArduinoSerialSink Serial;
