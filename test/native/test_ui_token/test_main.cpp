// test_ui_token -- native doctest suite for the shared /uitoken table
// Constraints:
// - Hardware-free and deterministic: every time is a literal u32 ms, the
//   secret is fixed, and no lock exists because the table never takes one.
// - Compiles flagship_p4/src/hub/UiTokenTable.h itself, the one copy the board
//   and the sim both build, so a slot or rate-gate change is caught here first.
// See: flagship_p4/src/hub/UiTokenTable.h, bd val-sf7.3

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <cstdint>
#include <optional>
#include <span>

// Named directly so the library finder adds lib/valence; it does not follow
// the relative include below.
#include "valence/core/crypto.hpp"

#include "../../../flagship_p4/src/hub/UiTokenTable.h"

using valence::SoftwareCrypto;
using valence::UiTokenTable;

namespace {

UiTokenTable::Secret fixedSecret() {
    UiTokenTable::Secret s{};
    for (size_t i = 0; i < s.size(); ++i) s[i] = std::byte(i * 7 + 1);
    return s;
}

// The whole host mint path minus the lock: claim, derive, install.
std::optional<UiTokenTable::Token> mint(UiTokenTable& t, uint32_t now) {
    const auto counter = t.claimMint(now);
    if (!counter) return std::nullopt;
    const UiTokenTable::Token tok = UiTokenTable::derive(fixedSecret(), *counter, now);
    t.install(tok, now);
    return tok;
}

std::span<const std::byte> bytes(const UiTokenTable::Token& t) { return std::span<const std::byte>(t); }

const SoftwareCrypto kCmp{};

}  // namespace

TEST_CASE("rate gate: one mint per 250 ms, a refused mint changes nothing") {
    UiTokenTable t;
    REQUIRE(mint(t, 1000));
    CHECK_FALSE(t.claimMint(1249));
    CHECK_FALSE(t.claimMint(1100));
    // The refusals did not move the gate: 250 ms after the FIRST mint opens it.
    CHECK(t.claimMint(1250));
}

TEST_CASE("rate gate: the very first mint is never refused, even at t = 0") {
    UiTokenTable t;
    CHECK(t.claimMint(0));
    CHECK_FALSE(t.claimMint(10));
}

TEST_CASE("rate gate holds across the u32 millisecond wrap") {
    UiTokenTable t;
    REQUIRE(t.claimMint(0xFFFFFFF0u));
    CHECK_FALSE(t.claimMint(0x00000010u));   // 32 ms later, across the wrap
    CHECK(t.claimMint(0x000000EAu));         // exactly 250 ms later
}

TEST_CASE("derive: HMAC over counter LE || now LE, deterministic and counter-unique") {
    const auto a = UiTokenTable::derive(fixedSecret(), 1, 5000);
    const auto b = UiTokenTable::derive(fixedSecret(), 1, 5000);
    const auto c = UiTokenTable::derive(fixedSecret(), 2, 5000);
    CHECK(a == b);
    CHECK(a != c);
    // First 16 bytes of HMAC-SHA256 over the documented 8-byte material.
    const std::array<std::byte, 8> material{std::byte{1}, std::byte{0}, std::byte{0}, std::byte{0},
                                            std::byte{0x88}, std::byte{0x13}, std::byte{0}, std::byte{0}};
    const auto secret = fixedSecret();
    const auto mac = valence::hmacSha256(std::span<const std::byte>(secret),
                                         std::span<const std::byte>(material));
    for (size_t i = 0; i < UiTokenTable::kTokenBytes; ++i) CHECK(a[i] == mac[i]);
}

TEST_CASE("consume: single use") {
    UiTokenTable t;
    const auto tok = mint(t, 1000);
    REQUIRE(tok);
    CHECK(t.consume(bytes(*tok), 1500, kCmp));
    CHECK_FALSE(t.consume(bytes(*tok), 1501, kCmp));
}

TEST_CASE("consume: wrong length and unknown tokens are refused and consume nothing") {
    UiTokenTable t;
    const auto tok = mint(t, 1000);
    REQUIRE(tok);
    CHECK_FALSE(t.consume(bytes(*tok).first(15), 1100, kCmp));
    UiTokenTable::Token forged = *tok;
    forged[0] = std::byte(uint8_t(forged[0]) ^ 0x01);
    CHECK_FALSE(t.consume(bytes(forged), 1100, kCmp));
    CHECK(t.consume(bytes(*tok), 1100, kCmp));
}

TEST_CASE("consume: a token dies at exactly its TTL") {
    UiTokenTable t;
    const auto tok = mint(t, 1000);
    REQUIRE(tok);
    UiTokenTable u = t;
    CHECK(u.consume(bytes(*tok), 1000 + UiTokenTable::kTtlMs - 1, kCmp));
    CHECK_FALSE(t.consume(bytes(*tok), 1000 + UiTokenTable::kTtlMs, kCmp));
}

TEST_CASE("consume: expiry is wrap-safe") {
    UiTokenTable t;
    const uint32_t mintAt = 0xFFFFFFFFu - 1000;   // expires after the wrap
    const auto tok = mint(t, mintAt);
    REQUIRE(tok);
    CHECK(t.consume(bytes(*tok), 5000, kCmp));    // ~6 s later, past the wrap, still live
}

TEST_CASE("install: a fifth live token evicts the oldest, never a newer one") {
    UiTokenTable t;
    std::array<UiTokenTable::Token, 5> toks{};
    for (uint32_t i = 0; i < 5; ++i) {
        const auto tok = mint(t, 1000 + i * UiTokenTable::kMinIntervalMs);
        REQUIRE(tok);
        toks[i] = *tok;
    }
    const uint32_t now = 3000;
    CHECK_FALSE(t.consume(bytes(toks[0]), now, kCmp));
    for (size_t i = 1; i < toks.size(); ++i) CHECK(t.consume(bytes(toks[i]), now, kCmp));
}

TEST_CASE("install: a used slot is reclaimed before any live token is evicted") {
    UiTokenTable t;
    std::array<UiTokenTable::Token, 4> toks{};
    for (uint32_t i = 0; i < 4; ++i) {
        const auto tok = mint(t, 1000 + i * UiTokenTable::kMinIntervalMs);
        REQUIRE(tok);
        toks[i] = *tok;
    }
    REQUIRE(t.consume(bytes(toks[2]), 2500, kCmp));   // slot 2 is now used
    const auto fifth = mint(t, 3000);
    REQUIRE(fifth);
    for (size_t i : {0u, 1u, 3u}) CHECK(t.consume(bytes(toks[i]), 3100, kCmp));
    CHECK(t.consume(bytes(*fifth), 3100, kCmp));
}
