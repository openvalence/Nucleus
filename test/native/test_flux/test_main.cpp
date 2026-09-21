// Flux -- hardware-free core suite. The heartbeat-gate contract is the
// safety-relevant part (a frozen core MUST freeze the LEDs), so it gets the
// most coverage; the two-axis arbiter, boot rainbow, pulse layering, blink
// timing, crossfade, and mode math follow.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <vector>

#include "flux/flux_core.hpp"

using namespace flux;

namespace {

struct FakeStrip final : IGlowOutput {
    size_t n;
    std::vector<Rgb> px;
    int shows = 0;
    explicit FakeStrip(size_t count) : n(count), px(count) {}
    size_t pixelCount() const override { return n; }
    void set(size_t i, Rgb c) override { px[i] = c; }
    void show() override { ++shows; }
};

// Pump past the crossfade so px holds the steady render of the current pair.
void settle(GlowEngine& g, uint32_t& t, uint32_t ms = kCrossfadeMs + 50) {
    for (uint32_t end = t + ms; t < end; t += 10) g.update(t);
}

}  // namespace

TEST_CASE("arbiter: highest Status wins; Safety wins ties; Nominal is the quiet floor") {
    FakeStrip strip(1);
    GlowEngine g(strip);

    CHECK(g.currentStatus() == Status::Nominal);
    g.set(System::Motion, Status::Working);
    CHECK(g.currentSystem() == System::Motion);
    CHECK(g.currentStatus() == Status::Working);

    // Higher Status takes the lamp regardless of system order.
    g.set(System::Link, Status::Degraded);
    CHECK(g.currentSystem() == System::Link);

    // Tie at Urgent: Safety is the later system, so it wins the tie.
    g.set(System::Flash, Status::Urgent);
    g.set(System::Safety, Status::Urgent);
    CHECK(g.currentSystem() == System::Safety);

    g.set(System::Safety, Status::Nominal);
    CHECK(g.currentSystem() == System::Flash);

    g.set(System::Flash, Status::Nominal);
    g.set(System::Link, Status::Nominal);
    CHECK(g.currentSystem() == System::Motion);
    g.set(System::Motion, Status::Nominal);
    CHECK(g.currentStatus() == Status::Nominal);
}

TEST_CASE("quiet floor renders green; a speaking system renders its own color") {
    FakeStrip strip(1);
    GlowEngine g(strip);
    uint32_t t = 0;
    settle(g, t, 5000);
    // All-Nominal: green floor (breathing, so green channel dominates always).
    CHECK(strip.px[0].g > strip.px[0].r);
    CHECK(strip.px[0].g > strip.px[0].b);

    g.set(System::Safety, Status::Urgent);
    settle(g, t);
    // Red, and blink-ON at phase start after the pair change reset animMs.
    CHECK(strip.px[0].r > 200);
    CHECK(strip.px[0].g == 0);
}

TEST_CASE("boot rainbow holds until every required system reports ready") {
    FakeStrip strip(1);
    GlowEngine g(strip);
    g.requireReady(uint8_t((1u << uint8_t(System::Motion)) |
                           (1u << uint8_t(System::Link))));
    CHECK(g.booting());

    // Rainbow ignores asserted states entirely, even Safety.
    g.set(System::Safety, Status::Urgent);
    uint32_t t = 0;
    Rgb seen[3] = {};
    settle(g, t, 400);
    seen[0] = strip.px[0];
    settle(g, t, 800);
    seen[1] = strip.px[0];
    settle(g, t, 800);
    seen[2] = strip.px[0];
    // Hue moves: three samples across the cycle cannot all match.
    const bool allSame = seen[0].r == seen[1].r && seen[1].r == seen[2].r &&
                         seen[0].g == seen[1].g && seen[1].g == seen[2].g &&
                         seen[0].b == seen[1].b && seen[1].b == seen[2].b;
    CHECK(!allSame);

    g.markReady(System::Motion);
    CHECK(g.booting());          // one of two: still booting
    g.markReady(System::Motion); // idempotent
    CHECK(g.booting());
    g.markReady(System::Link);
    CHECK(!g.booting());

    settle(g, t);
    CHECK(strip.px[0].r > 200);  // the asserted Safety Urgent shows now
    CHECK(strip.px[0].g == 0);
}

TEST_CASE("pulse overlays the steady render but never Ceremony or Urgent") {
    FakeStrip strip(1);
    GlowEngine g(strip);
    uint32_t t = 0;
    settle(g, t, 5000);

    // Quiet floor: a Motion ack pulse takes the pixel (amber: r>g, b=0).
    g.pulse(System::Motion, 100);
    g.update(t += 10);
    CHECK(strip.px[0].r > 200);
    CHECK(strip.px[0].b == 0);
    // It expires: well after 100 ms the green floor is back.
    settle(g, t, 300);
    CHECK(strip.px[0].g > strip.px[0].r);

    // Severity layer: Urgent is never covered by a pulse.
    g.set(System::Safety, Status::Urgent);
    settle(g, t);
    g.pulse(System::Motion, 100);
    g.update(t += 10);
    CHECK(strip.px[0].g == 0);   // still pure red, no amber leak
}

TEST_CASE("blink: ON for period_ms, OFF for half of it (operator ruling)") {
    FakeStrip strip(1);
    GlowEngine g(strip);
    uint32_t t = 0;
    settle(g, t, 1000);                        // establish the floor
    g.set(System::Safety, Status::Degraded);   // blink on=1200, off=600
    // The pair change resets phase to 0; skip the crossfade while staying
    // inside the 1200 ms ON window, then count duty over one 1800 ms cycle.
    settle(g, t, kCrossfadeMs + 10);
    int on = 0, samples = 0;
    for (uint32_t end = t + 1800; t < end; t += 50, ++samples) {
        g.update(t);
        if (strip.px[0].r > 100) ++on;
    }
    // 1200 of 1800 ms ON = 2/3 duty; allow slop for the sampled edges.
    CHECK(on > samples / 2);
    CHECK(on < samples);
}

TEST_CASE("heartbeat gate: a silent source freezes the frame exactly") {
    FakeStrip strip(1);
    GlowEngine g(strip);
    HeartbeatSource* core0 = g.addHeartbeat(150);
    HeartbeatSource* core1 = g.addHeartbeat(150);
    REQUIRE(core0 != nullptr);
    REQUIRE(core1 != nullptr);

    g.set(System::Motion, Status::Working);

    // Both cores pulsing: animation runs, frames latch.
    uint32_t t = 0;
    for (; t < 1000; t += 10) {
        core0->pulse();
        core1->pulse();
        g.update(t);
    }
    CHECK(!g.frozen());

    // core1 goes silent: animation runs for up to staleMs more, then the
    // engine freezes. Capture the frame AFTER the freeze engages -- from that
    // point it must not move by a single count.
    for (; t < 1400; t += 10) {
        core0->pulse();
        g.update(t);
    }
    CHECK(g.frozen());
    int showsFrozen = strip.shows;
    Rgb frameFrozen = strip.px[0];
    for (; t < 2000; t += 10) {
        core0->pulse();
        g.update(t);
    }
    CHECK(g.frozen());
    CHECK(strip.px[0].r == frameFrozen.r);
    CHECK(strip.px[0].g == frameFrozen.g);
    CHECK(strip.px[0].b == frameFrozen.b);
    CHECK(strip.shows == showsFrozen);       // frozen means NOTHING shown

    // Recovery: the silent core resumes, animation continues.
    for (; t < 3000; t += 10) {
        core0->pulse();
        core1->pulse();
        g.update(t);
    }
    CHECK(!g.frozen());
    CHECK(strip.shows > showsFrozen + 50);
}

TEST_CASE("no heartbeats registered: engine never freezes") {
    FakeStrip strip(1);
    GlowEngine g(strip);
    for (uint32_t t = 0; t < 1000; t += 10) g.update(t);
    CHECK(!g.frozen());
    CHECK(strip.shows > 50);
}

TEST_CASE("transition: double-blip announces the incoming pair, then settles") {
    FakeStrip strip(1);
    GlowEngine g(strip);
    uint32_t t = 0;
    settle(g, t, 5000);           // quiet green floor
    CHECK(strip.px[0].g > strip.px[0].r);

    g.set(System::Safety, Status::Urgent);
    const uint32_t t0 = t;        // intro elapsed = t - t0 - 10
    g.update(t += 10);
    // Blip 1: full-brightness incoming color, immediately.
    CHECK(strip.px[0].r == 255);
    CHECK(strip.px[0].g == 0);
    // The black gap between blips (elapsed ~80 ms).
    for (; t < t0 + 100; t += 10) g.update(t);
    CHECK(strip.px[0].r == 0);
    CHECK(strip.px[0].g == 0);
    // Blip 2 (elapsed ~140 ms).
    for (; t < t0 + 160; t += 10) g.update(t);
    CHECK(strip.px[0].r == 255);
    // Settled into the steady effect well after intro + fade.
    settle(g, t, 600);
    CHECK(strip.px[0].r > 200);
    CHECK(strip.px[0].g == 0);
}

TEST_CASE("brightness ceiling scales output, zero blacks out") {
    FakeStrip strip(1);
    GlowEngine g(strip);
    g.set(System::Safety, Status::Latched);   // solid red, no animation
    uint32_t t = 0;
    settle(g, t);
    CHECK(strip.px[0].r > 200);

    g.setBrightness(64);
    g.update(t += 10);
    CHECK(strip.px[0].r < 100);
    CHECK(strip.px[0].r > 20);

    g.setBrightness(0);
    g.update(t += 10);
    CHECK(strip.px[0].r == 0);
}

TEST_CASE("Rgb::lerp exact at endpoints; luma orders colors sanely") {
    Rgb a{0, 0, 0}, b{200, 100, 50};
    Rgb lo = Rgb::lerp(a, b, 0), hi = Rgb::lerp(a, b, 255);
    CHECK(lo.r == 0);
    CHECK(hi.r == 200);
    CHECK(hi.g == 100);
    CHECK(hi.b == 50);
    CHECK(Rgb{255, 255, 255}.luma() > Rgb{40, 40, 40}.luma());
    CHECK(Rgb{0, 255, 0}.luma() > Rgb{0, 0, 255}.luma());
}
