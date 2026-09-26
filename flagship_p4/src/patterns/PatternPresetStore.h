#pragma once

// PatternPresetStore -- the `pattern.frayd` preset store behind 0x5220: fixed
// slots of {name, opaque payload} plus the roster generation
// Constraints:
// - HARDWARE-FREE, fixed capacity, no heap. It knows nothing about what a
//   payload MEANS (SPEC §8.7: payloads are opaque); PatternSettings owns that.
// - OWNED BY THE HUB TASK (the delegate): the CRUD intents and readBlob() both
//   run there, so there is no lock and none is wanted (T5).
// - PERSISTED AS ONE BLOB (encode/decode below), all-or-nothing: magic,
//   version byte, the generation, then every slot, decoded only at its exact
//   length. The generation rides WITH the items: a client's cached enumeration
//   is keyed on it, so a restart that reset it beside surviving items would
//   leave that cache stale without a signal. Storage and debounce are the
//   composition's (ValenceHub.cpp NVS key "presets"; the twin's state file).
// - Every mutation bumps generation(): the roster's "re-enumerate" signal and
//   the generation readBlob() answers with.
// See: ValenceCatalog.h (pattern-presets, pattern-presets-roster),
// ValenceDevice.cpp (the pattern-presets-cmd verbs and readBlob)

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <string_view>

#include "PatternSettings.h"

namespace valence {

class PatternPresetStore {
public:
    static constexpr uint8_t kCapacity = 24;
    static constexpr uint8_t kNameMax  = 32;   // bytes including the NUL
    static constexpr uint8_t kPayloadBytes = uint8_t(PatternSettings::kPresetPayloadBytes);

    using Payload = PatternSettings::PresetPayload;

    struct Slot {
        std::array<char, kNameMax> name{};   // NUL-padded; name[0] == 0 means empty
        Payload payload{};

        bool used() const { return name[0] != '\0'; }
        std::string_view nameView() const { return std::string_view(name.data()); }
    };

    // nullptr for an out-of-range or EMPTY slot.
    const Slot* slot(uint8_t i) const {
        return (i < kCapacity && _slots[i].used()) ? &_slots[i] : nullptr;
    }
    uint16_t generation() const { return _generation; }

    uint8_t count() const {
        uint8_t n = 0;
        for (const Slot& s : _slots) n = uint8_t(n + (s.used() ? 1 : 0));
        return n;
    }

    // The slot IS the address: the client picks it. Overwrites. false on an
    // out-of-range slot or a name that is empty or does not fit.
    bool save(uint8_t i, std::string_view name, const Payload& payload) {
        if (i >= kCapacity || !nameFits(name)) return false;
        Slot& s = _slots[i];
        writeName(s, name);
        s.payload = payload;
        ++_generation;
        return true;
    }

    bool rename(uint8_t i, std::string_view name) {
        if (i >= kCapacity || !_slots[i].used() || !nameFits(name)) return false;
        writeName(_slots[i], name);
        ++_generation;
        return true;
    }

    bool remove(uint8_t i) {
        if (i >= kCapacity || !_slots[i].used()) return false;
        _slots[i].name.fill('\0');
        _slots[i].payload.fill(0);
        ++_generation;
        return true;
    }

    // ---- persistence blob ----
    static constexpr uint32_t kBlobMagic   = 0x56505253u;  // "VPRS"
    static constexpr uint8_t  kBlobVersion = 1;            // bump on ANY layout change
    // magic 4, version 1, generation 2, then per slot the NUL-padded name and
    // the payload
    static constexpr size_t kBlobBytes = 4 + 1 + 2 + size_t(kCapacity) * (kNameMax + kPayloadBytes);

    // Returns bytes written: kBlobBytes, or 0 when `out` is too small.
    size_t encode(std::span<std::byte> out) const {
        if (out.size() < kBlobBytes) return 0;
        size_t n = 0;
        put(out, n, &kBlobMagic, sizeof(kBlobMagic));
        put(out, n, &kBlobVersion, sizeof(kBlobVersion));
        put(out, n, &_generation, sizeof(_generation));
        for (const Slot& s : _slots) {
            put(out, n, s.name.data(), kNameMax);
            put(out, n, s.payload.data(), kPayloadBytes);
        }
        return n;
    }

    // All-or-nothing: every slot's name is checked BEFORE anything is copied,
    // so a false return leaves the store exactly as it was. Payload bytes are
    // opaque here; a load clamps them (PatternSettings::applyPreset).
    bool decode(std::span<const std::byte> in) {
        if (in.size() != kBlobBytes) return false;
        uint32_t magic = 0;
        uint8_t version = 0;
        std::memcpy(&magic, in.data(), sizeof(magic));
        std::memcpy(&version, in.data() + 4, sizeof(version));
        if (magic != kBlobMagic || version != kBlobVersion) return false;
        constexpr size_t kSlotsAt = 7;
        constexpr size_t kStride = size_t(kNameMax) + kPayloadBytes;
        for (size_t i = 0; i < kCapacity; ++i) {
            std::array<char, kNameMax> name{};
            std::memcpy(name.data(), in.data() + kSlotsAt + i * kStride, kNameMax);
            if (!storedNameValid(name)) return false;
        }
        std::memcpy(&_generation, in.data() + 5, sizeof(_generation));
        for (size_t i = 0; i < kCapacity; ++i) {
            const std::byte* at = in.data() + kSlotsAt + i * kStride;
            std::memcpy(_slots[i].name.data(), at, kNameMax);
            std::memcpy(_slots[i].payload.data(), at + kNameMax, kPayloadBytes);
        }
        return true;
    }

private:
    static void put(std::span<std::byte> out, size_t& n, const void* src, size_t len) {
        std::memcpy(out.data() + n, src, len);
        n += len;
    }
    // Empty (all NUL) or a name nameFits() would accept, NUL-padded to the end:
    // exactly the two shapes writeName() and remove() leave behind.
    static bool storedNameValid(const std::array<char, kNameMax>& name) {
        size_t len = 0;
        while (len < kNameMax && name[len] != '\0') ++len;
        if (len == kNameMax) return false;
        for (size_t k = len; k < kNameMax; ++k)
            if (name[k] != '\0') return false;
        return true;
    }

    static bool nameFits(std::string_view name) {
        return !name.empty() && name.size() < kNameMax && name.find('\0') == std::string_view::npos;
    }
    static void writeName(Slot& s, std::string_view name) {
        s.name.fill('\0');
        for (size_t k = 0; k < name.size(); ++k) s.name[k] = name[k];
    }

    std::array<Slot, kCapacity> _slots{};
    uint16_t _generation = 1;
};

}  // namespace valence
