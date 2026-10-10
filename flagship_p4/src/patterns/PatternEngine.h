#pragma once

// PatternEngine -- the stroke scheduler both generators share, and the two
// generators: ClassicGenerator (the seven StrokeEngine patterns) and
// AdvancedGenerator (the fray-d set with its modulators), each turning its
// own program into one MotionIntent per half-stroke
// Constraints:
// - HARDWARE-FREE and LIFTABLE: time enters as tick()'s argument, the motion
//   plane's state as PatternInputs, and every stroke leaves as a returned
//   MotionIntent the HOST submits through motionSubmit(). The engine never
//   touches the arbiter, the emitter or a queue (architecture.md section 2:
//   input sources submit intents, the arbiter is the sole caller).
// - TWO SOURCES, NEVER A MODE (RFC-093): each generator is its own instance
//   with its own run flag, stroke count and schedule, and stamps its own
//   MotionSource on every intent. The arbiter, not this file, keeps them off
//   the rail together.
// - EVENT-DRIVEN, NEVER CLOCKED: one intent per half-stroke, planned by the
//   arbiter at arrival. tick() only asks "is the next half-stroke due"; it
//   computes no positions on a clock.
// - A DWELL IS A HOLD SEGMENT (RFC-095, SPEC 9.6): the bound re-commanded as
//   a timed intent, never a silent gap. The plan strip then shows a live plan
//   whose start is its end, and kinetic's activity clock keeps running, so a
//   long dwell is neither a stall to a client nor a cold start to the next
//   half. PAUSE freezes the dwell: what was not yet held is owed on resume.
// - OWNED BY ONE TASK. apply() and tick() run on the host's pattern task (the
//   sim's one thread). Settings arrive as whole copies (PatternSettings.h).
// - The vendored patterns read millis(); ClassicGenerator sets the Arduino
//   adapter's clock from now_us before every call into one. One classic
//   generator per task.
// - ClassicGenerator holds seven pattern objects (~0.8 KB). Host both at file
//   scope, never as stack locals.
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

// The motion plane's state as a generator gates on it, read by the host from
// one motionCensus() per tick.
struct PatternInputs {
    bool  homed         = false;
    bool  estop         = false;
    bool  paused        = false;
    bool  stream_active = false;   // a Stream-source plan is live: yield to it
    float position_mm   = 0.0f;
    float velocity_mm_s = 0.0f;
    // The input set the planner plans a stroke under, overrides applied
    // (census input_*max); 0 bounds nothing.
    float vmax_mm_s     = 0.0f;
    float amax_mm_s2    = 0.0f;
    float jmax_mm_s3    = 0.0f;
};

// ---- the scheduler ----------------------------------------------------------

class PatternEngine {
public:
    // A run false -> true transition restarts the stroke count, so every
    // start opens with an in-stroke and a fresh modulator cycle.
    void apply(const PatternSettings& s);

    // At most one intent per call: the next half-stroke when it is due, or the
    // brake when the generator stops wanting motion mid-stroke.
    std::optional<MotionIntent> tick(uint64_t now_us, const PatternInputs& in);

    // True while this generator is driving: running, every gate open, and a
    // speed that moves. 0x1100's gen_running flag is either generator's.
    bool active() const { return _active; }

    // When tick() next has work, 0 when idle. A host sleeps until then.
    uint64_t nextDueUs() const { return _active ? _due_us : 0; }

    uint32_t strokeIndex() const { return _stroke_index; }

protected:
    explicit PatternEngine(MotionSource source) : _source(source) {}
    // Never destroyed through the base.
    ~PatternEngine() = default;

    struct Stroke {
        bool     moves = false;       // false: a zero-travel half-stroke, nothing to send
        float    target_mm = 0.0f;
        uint32_t duration_us = 0;
        uint64_t hold_us = 0;         // dwell at target once it lands; read only when moves
    };

    // This generator's own run/stop flag in the shared settings.
    virtual bool running(const PatternSettings& s) const = 0;
    // The knob set moves the machine at all (a speed above zero).
    virtual bool moves(const PatternSettings& s) const = 0;
    // The next half-stroke. nullopt: nothing due yet (a StopNGo pause), or
    // the pattern holds.
    virtual std::optional<Stroke> next(uint64_t now_us, const PatternInputs& in) = 0;

    float fromMm(const PatternInputs& in) const { return _have_prev ? _prev_target_mm : in.position_mm; }

    PatternSettings _s{};
    uint32_t _stroke_index = 0;

private:
    bool wantsMotion(const PatternInputs& in) const;
    MotionIntent brake(const PatternInputs& in) const;

    const MotionSource _source;
    bool     _active = false;
    uint64_t _due_us = 0;          // when the next half-stroke is due
    uint64_t _stroke_end_us = 0;   // when the half-stroke in flight lands
    bool     _holding = false;     // the intent in flight is a dwell's hold segment
    uint64_t _hold_owed_us = 0;    // dwell not yet sent: after a landing, the
                                   // remainder past one segment's cap, or
                                   // what a PAUSE froze
    bool     _have_prev = false;   // _prev_target_mm is the last stroke's target
    float    _prev_target_mm = 0.0f;
};

// ---- the two generators -----------------------------------------------------

// Source Pattern: plays PatternSettings' classic set, runs on `running`.
class ClassicGenerator final : public PatternEngine {
public:
    ClassicGenerator() : PatternEngine(MotionSource::Pattern) {}

private:
    bool running(const PatternSettings& s) const override { return s.running; }
    bool moves(const PatternSettings& s) const override { return s.speed > 0.0f && s.stroke > 0.0f; }
    std::optional<Stroke> next(uint64_t now_us, const PatternInputs& in) override;
    Pattern& patternAt(uint8_t idx);

    SimpleStroke    _simple{"Simple Stroke"};
    TeasingPounding _teasing{"Teasing Pounding"};
    RoboStroke      _robo{"Robo Stroke"};
    HalfnHalf       _half{"Half'n'Half"};
    Deeper          _deeper{"Deeper"};
    StopNGo         _stopngo{"Stop'n'Go"};
    Insist          _insist{"Insist"};
};

// Source Advanced: plays PatternSettings' advanced set and its modulators,
// runs on `adv_running`.
class AdvancedGenerator final : public PatternEngine {
public:
    AdvancedGenerator() : PatternEngine(MotionSource::Advanced) {}

private:
    bool running(const PatternSettings& s) const override { return s.adv_running; }
    bool moves(const PatternSettings& s) const override { return s.ap.master.value > 0; }
    std::optional<Stroke> next(uint64_t now_us, const PatternInputs& in) override;

    // The previous moving half's duration, 0 when there is none: a dwell's
    // stroke clock is this half plus that one (RFC-095).
    uint32_t _prev_half_us = 0;
};

}  // namespace valence
