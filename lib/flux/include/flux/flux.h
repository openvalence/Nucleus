// Flux -- Arduino output drivers over the hardware-free core.
//
// Constraints:
// - Drivers here are the generic, reusable ones (LEDC PWM color/mono). Board
//   wiring -- which pins, which states map to which fields, the heartbeat
//   sources -- lives in the firmware, NOT here.
// - There is NO ESP-IDF driver, and its absence is the rule, not a gap: no
//   board in this repo carries a pixel yet. A stamp with no LED injects
//   flux::NullGlowOutput from the core; the IDF driver lands with the
//   hardware it drives, never before it.
// - Native tests include flux_core.hpp directly and never reach this file.
#pragma once

#include "flux/flux_core.hpp"

#if defined(ARDUINO)
#include <Arduino.h>

namespace flux {

// gamma8 moved into flux_core.hpp (hardware-free, all boards share it).

// One true-color pixel on three LEDC PWM pins (discrete RGB LED). Handles
// active-low (current-sinking) LEDs by inverting duty.
class LedcRgbOutput final : public IGlowOutput {
public:
    LedcRgbOutput(uint8_t pinR, uint8_t pinG, uint8_t pinB, bool activeLow,
                  uint32_t freqHz = 5000)
        : _pins{pinR, pinG, pinB}, _activeLow(activeLow), _freq(freqHz) {}

    void begin() {
        for (uint8_t p : _pins) {
            ledcAttach(p, _freq, 8);
            ledcWrite(p, _activeLow ? 255 : 0);
        }
    }

    size_t pixelCount() const override { return 1; }
    void set(size_t, Rgb c) override { _pending = c; }
    void show() override {
        uint8_t d[3] = {gamma8(_pending.r), gamma8(_pending.g), gamma8(_pending.b)};
        for (int i = 0; i < 3; ++i)
            ledcWrite(_pins[i], _activeLow ? uint32_t(255 - d[i]) : uint32_t(d[i]));
    }

private:
    uint8_t _pins[3];
    bool _activeLow;
    uint32_t _freq;
    Rgb _pending{};
};

// One mono LED on an LEDC PWM pin, driven by the frame color's perceptual
// luma — an intensity shadow of the status pixel. Because every stock spec
// animates, this LED pulses in sympathy with the state color and freezes
// with the engine: it IS the heartbeat lamp.
class LedcMonoOutput final : public IGlowOutput {
public:
    LedcMonoOutput(uint8_t pin, bool activeHigh, uint32_t freqHz = 5000)
        : _pin(pin), _activeHigh(activeHigh), _freq(freqHz) {}

    void begin() {
        ledcAttach(_pin, _freq, 8);
        ledcWrite(_pin, _activeHigh ? 0 : 255);
    }

    size_t pixelCount() const override { return 1; }
    void set(size_t, Rgb c) override { _pending = c; }
    void show() override {
        uint8_t d = gamma8(_pending.luma());
        ledcWrite(_pin, _activeHigh ? uint32_t(d) : uint32_t(255 - d));
    }

private:
    uint8_t _pin;
    bool _activeHigh;
    uint32_t _freq;
    Rgb _pending{};
};

}  // namespace flux

#endif  // ARDUINO
