// test_trust_store -- native doctest suite for the trust ledger's persistence
// and the SPEC §12.3 power-cycle gesture
// Constraints:
// - Hardware-free: FakeNvs stands in for the "valence" NVS namespace, one map
//   entry per key, and counts every save so "written only on change" is
//   asserted, not assumed.
// - Each "reboot" is a fresh PairingManager and a fresh TrustStore over the
//   same FakeNvs, which is what a power cycle leaves behind.
// See: flagship_p4/src/hub/TrustStore.h, Valence SPEC.md §12.3, bd val-fvn

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <map>
#include <span>
#include <string>
#include <vector>

// Named directly so the library finder adds lib/valence; it does not follow
// the relative include below.
#include "valence/session/pairing.hpp"

#include "../../../flagship_p4/src/hub/TrustStore.h"

using valence::AccessLevel;
using valence::KeyLoad;
using valence::KeyLoadStatus;
using valence::PairingManager;
using valence::TrustBoot;
using valence::TrustStore;
namespace limits = valence::limits;

namespace {

class FakeNvs final : public valence::IKeyStore {
public:
    std::map<std::string, std::vector<std::byte>> keys;
    std::map<std::string, int> saves;
    bool failSaves = false;
    bool failLoads = false;

    KeyLoad load(const char* key, std::span<std::byte> scratch) override {
        const auto it = keys.find(key);
        if (it == keys.end()) return {};
        if (failLoads || it->second.size() > scratch.size()) return {KeyLoadStatus::Failed, {}};
        std::ranges::copy(it->second, scratch.begin());
        return {KeyLoadStatus::Loaded, scratch.first(it->second.size())};
    }
    bool save(const char* key, std::span<const std::byte> bytes) override {
        if (failSaves) return false;
        keys[key].assign(bytes.begin(), bytes.end());
        ++saves[key];
        return true;
    }
    int ledgerSaves() { return saves[valence::kLedgerKey]; }
    uint8_t counter() {
        const auto& v = keys[valence::kGestureKey];
        return v.size() == 1 ? uint8_t(v[0]) : 0xEE;
    }
};

PairingManager::PairedEntry entry(uint8_t id, AccessLevel role, const char* name) {
    PairingManager::PairedEntry e;
    e.instance_id.fill(std::byte(id));
    e.token.fill(std::byte(0xA0 + id));
    e.role = role;
    e.name.assign(name);
    e.kind.assign("phosphor");
    e.version.assign("0.9.1");
    return e;
}

// One power cycle: a fresh ledger and store over what the NVS kept.
struct Boot {
    PairingManager pm;
    TrustStore store;
    TrustBoot result;
    Boot(FakeNvs& nvs, bool powerOn = true) { result = store.boot(pm, nvs, powerOn); }
};

constexpr uint32_t kPastGesture = limits::pairing_gesture_max_uptime_ms;

}  // namespace

TEST_CASE("TS-01: a factory-fresh hub boots claimable and writes nothing for the ledger") {
    FakeNvs nvs;
    Boot b(nvs);
    CHECK_FALSE(b.result.ledgerLoaded);
    CHECK_FALSE(b.result.ledgerRejected);
    CHECK(b.result.claimable);
    CHECK(b.result.openWindow());
    CHECK(b.pm.presenceGrantRole() == AccessLevel::configure);
    CHECK(b.store.tick(b.pm, nvs, 5, false) == 0);
    CHECK(nvs.ledgerSaves() == 0);
}

TEST_CASE("TS-02: the ledger survives a reboot, tokens included, and ends claimability") {
    FakeNvs nvs;
    {
        Boot b(nvs);
        REQUIRE(b.pm.importEntry(entry(1, AccessLevel::configure, "owner")));
        REQUIRE(b.pm.importEntry(entry(2, AccessLevel::control, "remote")));
        CHECK(b.store.tick(b.pm, nvs, 100, false) == valence::kTrustLedgerWritten);
        CHECK(nvs.ledgerSaves() == 1);
    }
    Boot b(nvs);
    CHECK(b.result.ledgerLoaded);
    CHECK(b.result.paired == 2);
    CHECK_FALSE(b.result.claimable);
    const auto* owner = b.pm.entry(0);
    REQUIRE(owner != nullptr);
    CHECK(owner->role == AccessLevel::configure);
    CHECK(owner->name.sameAs("owner"));
    CHECK(owner->token == entry(1, AccessLevel::configure, "owner").token);
    // Adoption is not a change.
    CHECK(b.store.tick(b.pm, nvs, 5, false) == 0);
    CHECK(nvs.ledgerSaves() == 1);
}

TEST_CASE("TS-03: a ledger with only control entries is still factory-fresh (§12.3: zero configure tokens)") {
    FakeNvs nvs;
    {
        Boot b(nvs);
        REQUIRE(b.pm.importEntry(entry(2, AccessLevel::control, "remote")));
        b.store.tick(b.pm, nvs, 100, false);
    }
    Boot b(nvs);
    CHECK(b.result.ledgerLoaded);
    CHECK(b.result.claimable);
}

TEST_CASE("TS-04: written only on change, at most once per interval, held never dropped") {
    FakeNvs nvs;
    Boot b(nvs);
    REQUIRE(b.pm.importEntry(entry(1, AccessLevel::configure, "owner")));
    CHECK(b.store.tick(b.pm, nvs, 1000, false) == valence::kTrustLedgerWritten);

    // Dirty, but the bytes match what is stored (a HELLO that refreshed
    // nothing): no write.
    b.pm.observeHello(std::span<const std::byte>(entry(1, AccessLevel::configure, "").instance_id),
                      "phosphor", "owner", "0.9.1", true, 0);
    REQUIRE(b.pm.dirty());
    CHECK(b.store.tick(b.pm, nvs, 4000, false) == 0);
    CHECK(nvs.ledgerSaves() == 1);

    // A real change inside the interval is held, then lands.
    REQUIRE(b.pm.importEntry(entry(2, AccessLevel::control, "remote")));
    CHECK(b.store.tick(b.pm, nvs, 4500, false) == valence::kTrustLedgerWritten);
    REQUIRE(b.pm.revoke(std::span<const std::byte>(entry(2, AccessLevel::control, "").instance_id)));
    CHECK(b.store.tick(b.pm, nvs, 5000, false) == 0);
    CHECK(nvs.ledgerSaves() == 2);
    CHECK(b.store.tick(b.pm, nvs, 4500 + valence::kLedgerWriteMinIntervalMs, false) ==
          valence::kTrustLedgerWritten);
    CHECK(nvs.ledgerSaves() == 3);

    Boot after(nvs);
    CHECK(after.result.paired == 1);
}

TEST_CASE("TS-05: an OTA in flight holds the write until it ends") {
    FakeNvs nvs;
    Boot b(nvs);
    REQUIRE(b.pm.importEntry(entry(1, AccessLevel::configure, "owner")));
    CHECK(b.store.tick(b.pm, nvs, 100, true) == 0);
    CHECK(b.store.tick(b.pm, nvs, 9000, true) == 0);
    CHECK(nvs.ledgerSaves() == 0);
    CHECK(b.store.tick(b.pm, nvs, 9005, false) == valence::kTrustLedgerWritten);
}

TEST_CASE("TS-06: a failed save is retried at the interval") {
    FakeNvs nvs;
    Boot b(nvs);
    REQUIRE(b.pm.importEntry(entry(1, AccessLevel::configure, "owner")));
    nvs.failSaves = true;
    CHECK(b.store.tick(b.pm, nvs, 100, false) == valence::kTrustLedgerFailed);
    CHECK(b.store.tick(b.pm, nvs, 105, false) == 0);
    nvs.failSaves = false;
    CHECK(b.store.tick(b.pm, nvs, 100 + valence::kLedgerWriteMinIntervalMs, false) ==
          valence::kTrustLedgerWritten);
}

TEST_CASE("TS-07: a stored ledger that does not load leaves the hub unclaimable") {
    SUBCASE("rejected bytes") {
        FakeNvs nvs;
        nvs.keys[valence::kLedgerKey] = {std::byte(0xFF), std::byte(0x00)};
        Boot b(nvs);
        CHECK(b.result.ledgerRejected);
        CHECK_FALSE(b.result.claimable);
        CHECK(b.result.paired == 0);
    }
    SUBCASE("unreadable") {
        FakeNvs nvs;
        nvs.keys[valence::kLedgerKey] = {std::byte(0x80)};
        nvs.failLoads = true;
        Boot b(nvs);
        CHECK(b.result.ledgerRejected);
        CHECK_FALSE(b.result.claimable);
    }
}

TEST_CASE("TS-08: a full ledger at the registry's caps fits trust_ledger_max_bytes") {
    FakeNvs nvs;
    Boot b(nvs);
    const std::string longName(limits::trust_ledger_name_max_bytes, 'n');
    for (uint8_t i = 0; i < limits::paired_devices_max; ++i) {
        auto e = entry(uint8_t(i + 1), AccessLevel::configure, longName.c_str());
        e.kind.assign(std::string(limits::trust_ledger_kind_max_bytes, 'k'));
        e.version.assign(std::string(limits::client_ver_max_bytes, 'v'));
        e.firstSeen = 0xFFFFFFFFu;
        e.lastSeen = 0xFFFFFFFFu;
        REQUIRE(b.pm.importEntry(e));
    }
    CHECK(b.store.tick(b.pm, nvs, 100, false) == valence::kTrustLedgerWritten);
    CHECK(nvs.keys[valence::kLedgerKey].size() <= limits::trust_ledger_max_bytes);
    Boot after(nvs);
    CHECK(after.result.paired == limits::paired_devices_max);
}

TEST_CASE("TS-09: three short power-on boots arm the window on the fourth") {
    FakeNvs nvs;
    {
        Boot b(nvs);
        REQUIRE(b.pm.importEntry(entry(1, AccessLevel::configure, "owner")));
        b.store.tick(b.pm, nvs, 100, false);
        b.store.tick(b.pm, nvs, kPastGesture, false);   // a normal, long boot
        REQUIRE(nvs.counter() == 0);
    }
    for (uint32_t i = 1; i <= limits::pairing_gesture_boot_count; ++i) {
        Boot b(nvs);
        CHECK_FALSE(b.result.gesture);
        CHECK_FALSE(b.result.openWindow());
        CHECK(nvs.counter() == i);
    }
    Boot armed(nvs);
    CHECK(armed.result.gesture);
    CHECK(armed.result.openWindow());
    CHECK_FALSE(armed.result.claimable);
    // A configure token exists, so the window grants the default, not root.
    CHECK(armed.pm.presenceGrantRole() == AccessLevel::control);
    CHECK(nvs.counter() == 0);   // consumed
    Boot next(nvs);
    CHECK_FALSE(next.result.gesture);
}

TEST_CASE("TS-10: a boot that outlives the uptime limit restarts the count") {
    FakeNvs nvs;
    { Boot b(nvs); }
    { Boot b(nvs); }
    {
        Boot b(nvs);
        CHECK(nvs.counter() == 3);
        CHECK(b.store.tick(b.pm, nvs, kPastGesture - 1, false) == 0);
        CHECK(b.store.tick(b.pm, nvs, kPastGesture, false) == valence::kTrustGestureCleared);
        CHECK(nvs.counter() == 0);
        CHECK(b.store.tick(b.pm, nvs, kPastGesture + 5, false) == 0);   // once
    }
    Boot b(nvs);
    CHECK_FALSE(b.result.gesture);
    CHECK(nvs.counter() == 1);
}

TEST_CASE("TS-11: a non-power-on reset breaks the sequence") {
    FakeNvs nvs;
    { Boot b(nvs); }
    { Boot b(nvs); }
    { Boot b(nvs); }
    { Boot crash(nvs, false); CHECK_FALSE(crash.result.gesture); }
    CHECK(nvs.counter() == 0);
    Boot b(nvs);
    CHECK_FALSE(b.result.gesture);
}

TEST_CASE("TS-12: a corrupt counter delays a gesture, never fakes one") {
    FakeNvs nvs;
    nvs.keys[valence::kGestureKey] = {std::byte(0xFF)};
    Boot b(nvs);
    CHECK_FALSE(b.result.gesture);
    CHECK(nvs.counter() == 1);
}

TEST_CASE("TS-13: a planned reboot's flush writes a change held inside the interval, and nothing else") {
    FakeNvs nvs;
    Boot b(nvs);
    REQUIRE(b.pm.importEntry(entry(1, AccessLevel::configure, "owner")));
    CHECK(b.store.tick(b.pm, nvs, 1000, false) == valence::kTrustLedgerWritten);
    // Nothing pending: the flush writes nothing.
    CHECK(b.store.flush(b.pm, nvs, 1100) == 0);
    CHECK(nvs.ledgerSaves() == 1);
    // A change inside the interval: tick holds it, the flush lands it.
    REQUIRE(b.pm.importEntry(entry(2, AccessLevel::control, "remote")));
    CHECK(b.store.tick(b.pm, nvs, 1200, false) == 0);
    CHECK(b.store.flush(b.pm, nvs, 1300) == valence::kTrustLedgerWritten);
    CHECK(nvs.ledgerSaves() == 2);
    Boot after(nvs);
    CHECK(after.result.paired == 2);
}
