#pragma once

// PatternEngine -- the stroke generator: the seven StrokeEngine patterns and
// the fray-d advanced set, turned into one MotionIntent per half-stroke
// Constraints:
// - HARDWARE-FREE and LIFTABLE: time enters as tick()'s argument, the motion
//   plane's state as PatternInputs, and every stroke leaves as a returned
//   MotionIntent the HOST submits through motionSubmit(). The engine never
//   touches the arbiter, the emitter or a queue (architecture.md section 2:
//   input sources submit intents, the arbiter is the sole caller).
// - EVENT-DRIVEN, NEVER CLOCKED: one intent per half-stroke, planned by the
//   arbiter at arrival. tick() only asks "is the next half-stroke due"; it
//   computes no positions on a clock.
// - OWNED BY ONE TASK. apply() and tick() run on the host's pattern task (the
//   sim's one thread). Settings arrive as whole copies (PatternSettings.h).
// - The vendored patterns read millis(); tick() sets the Arduino adapter's
//   clock from now_us before every call into one. One engine per task.
// - Holds seven pattern objects (~0.8 KB). Host it at file scope, never as a
//   stack local.
// See: PatternSettings.h, ValencePattern.h, lib/strokeengine_patterns,
// .claude/rules/motion-control.md

#include <cstdint>
#include <optional>

#include "PatternSettings.h"
#include "motion/ValenceMotion.h"

// Vendored verbatim (lib/strokeengine_patterns/VENDORED.md): its one warning,
// Arduino's unsigned millis() against its own int deadline, is silenced at the
// include rather than patched. IDF builds with -Werror=all.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-compare"
#include "pattern.h"
#pragma GCC diagnostic pop

namespace valence {

// The motion plane's state as the generator gates on it, read by the host
// from one motionCensus() per tick.
struct PatternInputs {
    bool  homed         = false;
    bool  estop         = false;
    bool  paused        = false;
    bool  stream_active = false;   // a Stream-source plan is live: yield to it
    float position_mm   = 0.0f;
    float velocity_mm_s = 0.0f;
};

class PatternEngine {
public:
    // A running false -> true transition restarts the stroke count, so every
    // start opens with an in-stroke and a fresh lane cycle.
    void apply(const PatternSettings& s);

    // At most one intent per call: the next half-stroke when it is due, or the
    // brake when the generator stops wanting motion mid-stroke.
    std::optional<MotionIntent> tick(uint64_t now_us, const PatternInputs& in);

    // True while the generator is driving: running, every gate open, and a
    // speed that moves. 0x1100's gen_running flag.
    bool active() const { return _active; }

    // When tick() next has work, 0 when idle. A host sleeps until then.
    uint64_t nextDueUs() const { return _active ? _due_us : 0; }

    uint32_t strokeIndex() const { return _stroke_index; }

private:
    struct Stroke {
        bool     moves = false;       // false: a zero-travel half-stroke, nothing to send
        float    target_mm = 0.0f;
        uint32_t duration_us = 0;
    };

    bool wantsMotion(const PatternInputs& in) const;
    // nullopt: nothing due yet (a StopNGo pause), or the pattern holds.
    std::optional<Stroke> nextClassic(uint64_t now_us, const PatternInputs& in);
    std::optional<Stroke> nextAdvanced(const PatternInputs& in);
    MotionIntent brake(const PatternInputs& in) const;
    float fromMm(const PatternInputs& in) const { return _have_prev ? _prev_target_mm : in.position_mm; }
    Pattern& patternAt(uint8_t idx);

    PatternSettings _s{};

    SimpleStroke    _simple{"Simple Stroke"};
    TeasingPounding _teasing{"Teasing Pounding"};
    RoboStroke      _robo{"Robo Stroke"};
    HalfnHalf       _half{"Half'n'Half"};
    Deeper          _deeper{"Deeper"};
    StopNGo         _stopngo{"Stop'n'Go"};
    Insist          _insist{"Insist"};

    uint32_t _stroke_index = 0;
    bool     _active = false;
    uint64_t _due_us = 0;          // when the next half-stroke is due
    uint64_t _stroke_end_us = 0;   // when the half-stroke in flight lands
    bool     _have_prev = false;   // _prev_target_mm is the last stroke's target
    float    _prev_target_mm = 0.0f;
};

}  // namespace valence
