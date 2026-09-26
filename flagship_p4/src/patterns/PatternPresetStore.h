#pragma once

// PatternPresetStore -- the `pattern.frayd` preset store behind 0x5220: fixed
// slots of {name, opaque payload} plus the roster generation
// Constraints:
// - HARDWARE-FREE, fixed capacity, no heap. It knows nothing about what a
//   payload MEANS (SPEC §8.7: payloads are opaque); PatternSettings owns that.
// - OWNED BY THE HUB TASK (the delegate): the CRUD intents and readBlob() both
//   run there, so there is no lock and none is wanted (T5).
// - RAM ONLY. A reboot empties it. TODO(val-wcm): NVS persistence on the
//   board, one blob, all-or-nothing, debounced like the 0x1000 config.
// - Every mutation bumps generation(): the roster's "re-enumerate" signal and
//   the generation readBlob() answers with.
// See: ValenceCatalog.h (pattern-presets, pattern-presets-roster),
// ValenceDevice.cpp (the pattern-presets-cmd verbs and readBlob)

#include <array>
#include <cstdint>
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

private:
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
