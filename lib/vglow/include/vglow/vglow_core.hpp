// VGlow — hardware-free status-LED core.
//
// Philosophy (the three rules everything here serves):
//   1. CALLERS SPEAK SEMANTICS. Modules set a system's status
//      (glow.set(System::Motion, Status::Working)) and never pick colors or
//      patterns -- COLOR names the system, EFFECT names the status, and the
//      arbiter shows the most time-sensitive pair on whatever hardware the
//      board actually has. That mapping is the module's job, which is what
//      makes callers hardware-interchangeable.
//   2. THE LED IS A LIVENESS ORGAN. The engine has no task and no timer:
//      animation phase only advances inside update(), and only while every
//      registered heartbeat source (one per monitored core/task) has pulsed
//      recently. A frozen core freezes the animation mid-frame — a static
//      LED IS the failure indication. (NeoPixels latch their last color with
//      zero CPU, so this works all the way down.)
//   3. NOTHING HERE TOUCHES HARDWARE. Output is an injected IGlowOutput;
//      time is caller-supplied ms. The core runs identically on the S3, the
//      C5 nodes, and in the native doctest suite.
#pragma once

#include <cstddef>
#include <cstdint>

namespace vglow {

// ---- Color ------------------------------------------------------------------

struct Rgb {
    uint8_t r = 0, g = 0, b = 0;

    static constexpr Rgb black() { return {0, 0, 0}; }
    constexpr bool operator==(const Rgb&) const = default;

    // Integer lerp, t in [0,255]. Exact at both endpoints.
    static Rgb lerp(Rgb a, Rgb b8, uint8_t t) {
        auto mix = [t](uint8_t x, uint8_t y) {
            return uint8_t(x + ((int(y) - int(x)) * t) / 255);
        };
        return {mix(a.r, b8.r), mix(a.g, b8.g), mix(a.b, b8.b)};
    }

    // Perceptual luma (for mono outputs), 0..255.
    uint8_t luma() const {
        return uint8_t((uint16_t(r) * 54 + uint16_t(g) * 183 + uint16_t(b) * 19) >> 8);
    }
};

// Perceptual transfer (^2.2): LED radiance is linear in duty, the eye is
// logarithmic; every OUTPUT DRIVER applies this inverse curve while the
// core's shapes stay perceptual (ruling 2026-08-06). x^2.2 = x^2 * fifth
// root of x, Newton, so the hardware-free core needs no <cmath>. Not T4:
// first call is task context, no critical section.
inline uint8_t gamma8(uint8_t v) {
    static const uint8_t* table = [] {
        static uint8_t t[256];
        for (int i = 0; i < 256; ++i) {
            const double x = double(i) / 255.0;
            double r = 0.0;
            if (x > 0.0) {
                r = 0.5 + 0.5 * x;
                for (int k = 0; k < 24; ++k) {
                    const double r4 = r * r * r * r;
                    r = r - (r * r4 - x) / (5.0 * r4);  // r -> x^(1/5)
                }
            }
            t[i] = uint8_t(x * x * r * 255.0 + 0.5);
        }
        return t;
    }();
    return table[v];
}

// HSV→RGB for the rainbow/cycle mode. h in [0,255] wraps; s,v in [0,255].
inline Rgb hsv(uint8_t h, uint8_t s, uint8_t v) {
    if (s == 0) return {v, v, v};
    uint8_t region = h / 43;
    uint8_t rem = uint8_t((h - region * 43) * 6);
    uint8_t p = uint8_t((uint16_t(v) * (255 - s)) >> 8);
    uint8_t q = uint8_t((uint16_t(v) * (255 - ((uint16_t(s) * rem) >> 8))) >> 8);
    uint8_t t = uint8_t((uint16_t(v) * (255 - ((uint16_t(s) * (255 - rem)) >> 8))) >> 8);
    switch (region) {
        case 0: return {v, t, p};
        case 1: return {q, v, p};
        case 2: return {p, v, t};
        case 3: return {p, q, v};
        case 4: return {t, p, v};
        default: return {v, p, q};
    }
}

// ---- Output driver contract -------------------------------------------------

// A strip of N logical pixels. A dumb mono LED is a 1-pixel output whose
// driver maps Rgb -> luma() -> PWM duty; an RGB LED is 1 pixel of true
// color; a NeoPixel ring is N. show() latches the frame to hardware.
class IGlowOutput {
public:
    virtual ~IGlowOutput() = default;
    virtual size_t pixelCount() const = 0;
    virtual void set(size_t i, Rgb c) = 0;
    virtual void show() = 0;
};

// The output for a board that carries no pixel. It is a real driver, not a
// placeholder: the engine still arbitrates, still runs the liveness gate, and
// still reports what it WOULD render, so a stamp without an LED exercises the
// same code path the one with an LED will. Hardware-free, so it is also what
// the native suite injects. Board wiring never lives here.
class NullGlowOutput final : public IGlowOutput {
public:
    size_t pixelCount() const override { return 1; }
    void set(size_t, Rgb c) override { _last = c; }
    void show() override {}

    Rgb last() const { return _last; }

private:
    Rgb _last{};
};

// ---- The two-axis grammar (operator rulings 2026-08-06) ---------------------
// COLOR names the SYSTEM speaking; EFFECT names that system's STATUS. One
// pixel shows one pair: the arbiter takes the highest STATUS -- the
// time-sensitivity rank, T15 generalized from a hand-ordered enum into a
// rule -- with ties broken toward the higher System (Safety above all).
// All-Nominal renders the green quiet floor. During boot the pixel runs
// Rainbow until every system the board required has reported ready.
// PULSES overlay the steady render for ~100 ms (an ack blip in the pulsing
// system's color) but never override Ceremony or Urgent.

enum class System : uint8_t {
    Motion = 0,   // amber   -- motion plane INCLUDING the drive/Modbus bus
    Link,         // cyan    -- the UART bridge; same color on BOTH ends
    Network,      // blue    -- WiFi/WS (never speaks on a headless S3)
    Session,      // magenta -- sessions, auth, pairing ceremonies
    Flash,        // white   -- THIS device's firmware being written
    Safety,       // red     -- fault / e-stop; highest tie-break
    kCount_,
};
inline constexpr size_t kSystemCount = size_t(System::kCount_);

enum class Status : uint8_t {
    Nominal = 0,  // slow breathe -- background, nothing to say
    Latched,      // solid        -- waiting on the operator
    Working,      // fast breathe -- actively doing its job
    Degraded,     // slow blink   -- running but wrong
    Ceremony,     // blink        -- gone if missed (pairing window)
    Urgent,       // fast blink   -- exclusive, hands off
    kCount_,
};
inline constexpr size_t kStatusCount = size_t(Status::kCount_);

enum class GlowMode : uint8_t {
    Solid = 0,
    Breathe,    // colorA <-> colorB sine-ish lerp over period
    Blink,      // period_ms is the ON time; OFF is half of it (ruling: the
                // color stays readable, the off-gap is the punctuation)
    Rainbow,    // hue cycle over period (boot only)
};

struct GlowSpec {
    GlowMode mode = GlowMode::Solid;
    uint16_t period_ms = 1000;
};

inline constexpr Rgb colorOf(System s) {
    switch (s) {
        case System::Motion:  return {255, 140, 0};
        case System::Link:    return {0, 200, 255};
        case System::Network: return {0, 60, 255};
        case System::Session: return {255, 0, 90};
        case System::Flash:   return {255, 255, 255};
        case System::Safety:  return {255, 0, 0};
        default:              return {};
    }
}
// PURE green on purpose: the old {0,255,60} tint read as teal on a WS2812,
// too close to cyan=Link (operator eyeball, 2026-08-06). Hue separation is
// the product; aesthetics lost.
inline constexpr Rgb kQuietColor{0, 255, 0};

inline constexpr GlowSpec specOf(Status st) {
    switch (st) {
        case Status::Nominal:  return {GlowMode::Breathe, 3000};
        case Status::Latched:  return {GlowMode::Solid, 0};
        case Status::Working:  return {GlowMode::Breathe, 1400};
        case Status::Degraded: return {GlowMode::Blink, 1200};   // off 600
        case Status::Ceremony: return {GlowMode::Blink, 700};    // off 350
        case Status::Urgent:   return {GlowMode::Blink, 400};    // off 200
        default:               return {};
    }
}

// ---- Heartbeat gate ---------------------------------------------------------

// One counter per monitored loop. The owner of that loop bumps pulse() every
// iteration (an atomic-free relaxed increment is fine: any observed change
// proves liveness). The engine freezes animation when ANY registered source
// hasn't changed within its staleness window.
struct HeartbeatSource {
    volatile uint32_t counter = 0;
    void pulse() { counter = counter + 1; }
};

// ---- The engine -------------------------------------------------------------

inline constexpr size_t kMaxPixels = 16;
inline constexpr size_t kMaxHeartbeats = 4;
inline constexpr uint16_t kCrossfadeMs = 350;
// Transition announcement (operator ruling: state changes must POP): two
// full-brightness blips of the INCOMING pair's color with a black gap, then
// the crossfade emerges underneath. Applies to every status, Urgent included
// -- an estop announcing itself louder is correct. Boot rainbow is exempt.
inline constexpr uint16_t kIntroMs = 190;
inline constexpr uint16_t kIntroOn1End = 70;
inline constexpr uint16_t kIntroGapEnd = 120;

class GlowEngine {
public:
    explicit GlowEngine(IGlowOutput& out) : _out(out) {}

    // ---- Semantic surface (any task; single writer per system, u8 stores
    // are atomic on every target). Each system holds exactly one Status.
    void set(System sys, Status st) { _status[size_t(sys)] = uint8_t(st); }
    Status status(System sys) const { return Status(_status[size_t(sys)]); }

    // ---- Transient overlay (operator ruling): a short ack blip in the
    // system's color. Overrides the steady render but NEVER Ceremony/Urgent.
    void pulse(System sys, uint16_t ms = 100) {
        _pulseColor = colorOf(sys);
        _pulseRemainingMs = ms;   // u16 store; last writer wins, by design
    }

    // ---- Boot rainbow: rainbow until every required system reported ready.
    void requireReady(uint8_t systemsMask) { _readyPending = systemsMask; }
    void markReady(System sys) { _readyPending &= uint8_t(~(1u << uint8_t(sys))); }
    bool booting() const { return _readyPending != 0; }

    // What the pixel is showing (arbiter result), for tests and debug.
    System currentSystem() const { return winner().sys; }
    Status currentStatus() const { return winner().st; }

    // Global brightness ceiling, 0..255 (applied after everything else).
    void setBrightness(uint8_t b) { _brightness = b; }

    // ---- Heartbeat registration (call once per monitored loop at init).
    // Returns nullptr when full. staleMs: how long silence means "frozen".
    HeartbeatSource* addHeartbeat(uint16_t staleMs = 150) {
        if (_hbCount >= kMaxHeartbeats) return nullptr;
        _hb[_hbCount].staleMs = staleMs;
        return &_hb[_hbCount++].src;
    }

    bool frozen() const { return _frozen; }

    // ---- Pump. Call from ONE task (Core 0). Animation time advances only
    // while every heartbeat is alive; a stalled source freezes the frame
    // exactly where it is (rule 2 in the header).
    void update(uint32_t nowMs) {
        uint32_t dt = _hasTime ? (nowMs - _lastMs) : 0;
        _lastMs = nowMs;
        _hasTime = true;

        _frozen = anyHeartbeatStale(nowMs);
        if (_frozen) return;  // no phase advance, no show(): frame latches as-is

        _animMs += dt;

        const Pick w = winner();
        const uint16_t key = booting()
            ? 0xFFFF : uint16_t((uint16_t(w.sys) << 8) | uint16_t(w.st));
        if (key != _shownKey) {
            // Crossfade start: capture the outgoing frame as the fade origin.
            for (size_t i = 0; i < framePixels(); ++i) _fadeFrom[i] = _frame[i];
            _shownKey = key;
            _fadeRemainingMs = kCrossfadeMs;
            _animMs = 0;  // the new pair starts its pattern at phase 0
            if (!booting()) {
                _introColor = (w.st == Status::Nominal) ? kQuietColor : colorOf(w.sys);
                _introRemainingMs = kIntroMs;
            }
        }

        renderPick(w, _animMs);

        if (_fadeRemainingMs > 0) {
            uint16_t step = uint16_t(dt > _fadeRemainingMs ? _fadeRemainingMs : dt);
            _fadeRemainingMs = uint16_t(_fadeRemainingMs - step);
            uint8_t t = uint8_t(255 - (uint32_t(_fadeRemainingMs) * 255) / kCrossfadeMs);
            for (size_t i = 0; i < framePixels(); ++i)
                _frame[i] = Rgb::lerp(_fadeFrom[i], _frame[i], t);
        }

        // Pulse overlay: hard flash, no fade -- punctuation, not a state.
        // Severity-layered (operator ruling): Ceremony/Urgent never covered.
        if (_pulseRemainingMs > 0) {
            uint16_t step = uint16_t(dt > _pulseRemainingMs ? _pulseRemainingMs : dt);
            _pulseRemainingMs = uint16_t(_pulseRemainingMs - step);
            if (!booting() && w.st < Status::Ceremony)
                for (size_t i = 0; i < framePixels(); ++i) _frame[i] = _pulseColor;
        }

        // Transition announcement: the double-blip outranks everything below
        // it, including a pending ack pulse -- it IS the new state speaking.
        if (_introRemainingMs > 0) {
            const uint16_t elapsed = uint16_t(kIntroMs - _introRemainingMs);
            uint16_t step = uint16_t(dt > _introRemainingMs ? _introRemainingMs : dt);
            _introRemainingMs = uint16_t(_introRemainingMs - step);
            const bool on = elapsed < kIntroOn1End || elapsed >= kIntroGapEnd;
            const Rgb c = on ? _introColor : Rgb{0, 0, 0};
            for (size_t i = 0; i < framePixels(); ++i) _frame[i] = c;
        }

        for (size_t i = 0; i < framePixels(); ++i) _out.set(i, scale(_frame[i]));
        _out.show();
    }

private:
    struct Pick {
        System sys;
        Status st;
    };

    // Highest Status wins; ties go to the higher System (Safety is last, so
    // it wins every tie it enters). All-Nominal is the quiet floor.
    Pick winner() const {
        Pick best{System::Motion, Status::Nominal};
        for (size_t i = 0; i < kSystemCount; ++i) {
            const Status st = Status(_status[i]);
            if (uint8_t(st) >= uint8_t(best.st)) best = {System(i), st};
        }
        return best;
    }

    struct Heartbeat {
        HeartbeatSource src;
        uint16_t staleMs = 150;
        uint32_t lastSeen = 0;
        uint32_t lastChangeMs = 0;
        bool seeded = false;
    };

    size_t framePixels() const {
        size_t n = _out.pixelCount();
        return n < kMaxPixels ? n : kMaxPixels;
    }

    bool anyHeartbeatStale(uint32_t nowMs) {
        bool stale = false;
        for (size_t i = 0; i < _hbCount; ++i) {
            Heartbeat& h = _hb[i];
            uint32_t c = h.src.counter;
            if (!h.seeded || c != h.lastSeen) {
                h.seeded = true;
                h.lastSeen = c;
                h.lastChangeMs = nowMs;
            } else if (nowMs - h.lastChangeMs > h.staleMs) {
                stale = true;  // keep scanning: every source's bookkeeping stays fresh
            }
        }
        return stale;
    }

    // Triangle wave 0..255..0 over period, from _animMs.
    static uint8_t trianglePhase(uint32_t animMs, uint16_t period) {
        if (period == 0) return 255;
        uint32_t ph = (animMs % period) * 512u / period;  // 0..511
        return uint8_t(ph < 256 ? ph : 511 - ph);
    }

    // Smoothstep (3t^2 - 2t^3) of the triangle: zero slope at both turns, so
    // the breathe dwells at floor and crest (the linear triangle read harsh,
    // ruling 2026-08-06). Shape is perceptual; drivers' gamma8 does the rest.
    static uint8_t easedPhase(uint32_t animMs, uint16_t period) {
        const uint32_t t = trianglePhase(animMs, period);
        return uint8_t((t * t * (765u - 2u * t)) / 65025u);
    }

    static Rgb dim(Rgb c) { return {uint8_t(c.r / 18), uint8_t(c.g / 18), uint8_t(c.b / 18)}; }

    void renderPick(const Pick& w, uint32_t animMs) {
        const size_t n = framePixels();
        if (booting()) {  // rainbow until every required system is ready
            const uint16_t period = 2500;
            uint8_t h0 = uint8_t((animMs % period) * 255u / period);
            for (size_t i = 0; i < n; ++i)
                _frame[i] = hsv(uint8_t(h0 + (n > 1 ? i * 255 / n : 0)), 255, 255);
            return;
        }
        const bool quiet = (w.st == Status::Nominal);
        const Rgb a = quiet ? kQuietColor : colorOf(w.sys);
        const Rgb b = dim(a);
        GlowSpec s = specOf(w.st);
        // Flash blinks TWICE as fast as its status tempo (ruling 2026-08-06):
        // white is the do-not-power-off veto, and its urgency reads in the
        // rate. Same on/off ratio, half the scale.
        if (w.sys == System::Flash && s.mode == GlowMode::Blink)
            s.period_ms = uint16_t(s.period_ms / 2u);
        switch (s.mode) {
            case GlowMode::Solid:
                for (size_t i = 0; i < n; ++i) _frame[i] = a;
                break;
            case GlowMode::Breathe: {
                Rgb c = Rgb::lerp(b, a, easedPhase(animMs, s.period_ms));
                for (size_t i = 0; i < n; ++i) _frame[i] = c;
                break;
            }
            case GlowMode::Blink: {
                // ON for period_ms, OFF for half of it (operator ruling): the
                // color carries the message, the gap is the punctuation.
                const uint32_t cycle = uint32_t(s.period_ms) + s.period_ms / 2u;
                bool on = cycle == 0 || (animMs % cycle) < s.period_ms;
                Rgb c = on ? a : b;
                for (size_t i = 0; i < n; ++i) _frame[i] = c;
                break;
            }
            default:
                for (size_t i = 0; i < n; ++i) _frame[i] = a;
                break;
        }
    }

    Rgb scale(Rgb c) const {
        auto s = [this](uint8_t v) { return uint8_t((uint16_t(v) * (_brightness + 1)) >> 8); };
        return {s(c.r), s(c.g), s(c.b)};
    }

    IGlowOutput& _out;
    volatile uint8_t _status[kSystemCount] = {};
    Rgb _frame[kMaxPixels] = {};
    Rgb _fadeFrom[kMaxPixels] = {};
    Rgb _pulseColor{};
    uint16_t _pulseRemainingMs = 0;
    Rgb _introColor{};
    uint16_t _introRemainingMs = 0;
    uint16_t _shownKey = 0xFFFE;   // neither a valid pair nor the boot key
    Heartbeat _hb[kMaxHeartbeats];
    size_t _hbCount = 0;
    uint8_t _readyPending = 0;
    uint32_t _animMs = 0;
    uint32_t _lastMs = 0;
    uint16_t _fadeRemainingMs = 0;
    uint8_t _brightness = 255;
    bool _hasTime = false;
    bool _frozen = false;
};

}  // namespace vglow
