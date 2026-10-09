#pragma once

// SimCore -- the twin's machine: catalog, delegate, Hub, motion, patterns,
// motor switch, /uitoken table and persistence, with no socket and no clock
// of its own. Two fronts drive it: src/main.cpp (the native exe: WS, HTTP,
// UDP discovery, a wall clock) and wasm/integral.cpp (the in-process build:
// the host drives time and carries the bytes).
// Constraints:
// - ONE thread calls everything here, and pass() is the only tick (T5).
// - deviceNowUs() and geiger::hostNowMs() are the FRONT's: each front defines
//   them over the same clock it passes to begin().
// - One SimCore per process: SimMotion, SimPattern and SimMotorSwitch are
//   process singletons and Geiger has no removeSink, so a core is never
//   destroyed before exit.
// - Stored state is read BEFORE the Hub is constructed (the P4's hubBegin
//   order); a front never loads a blob after begin().
// See: sim/valencesim/README.md

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>

#include "SimUiToken.h"
#include "common/HostPlatform.h"
#include "common/SessionLog.h"
#include "geiger/geiger.h"
#include "hub/ValenceCatalog.h"
#include "hub/ValenceDevice.h"
#include "patterns/ValencePattern.h"

namespace valence {

// The board's three NVS keys, as blobs a front stores where it likes.
enum class SimBlob : uint8_t { cfg = 1, presets = 2, iid = 3 };

class ISimStore {
public:
    virtual ~ISimStore() = default;
    // The blob's bytes in `scratch`, or empty for absent or too large.
    virtual std::span<const std::byte> load(SimBlob b, std::span<std::byte> scratch) = 0;
    // False when the blob could not be kept (the core logs it).
    virtual bool save(SimBlob b, std::span<const std::byte> blob) = 0;
};

struct SimConfig {
    bool homed = false;
    bool uncommissioned = false;
    bool pairingWindow = false;
    bool motorSwitch = false;
    int mswFaultS = -1;
    std::optional<float> homeSenseAtMm;
    std::optional<float> railEndAtMm;
    uint32_t planDelayMs = 0;     // --plan-delay-ms (SimMotion.h)
    const char* storeName = "";   // names the store in log lines only
};

class SimCore {
public:
    static constexpr const char* kHubName = "Virtual";

    // False on a catalog that overflows or encodes to zero (logged to stderr).
    bool begin(const SimConfig& cfg, IClock& clock, ISimStore& store);

    // One motion pass at now_us; every 5 ms also one hub tick, with `pump`
    // (the front's transports: attach, detach, stall sweep) run before
    // hub.update.
    void pass(uint64_t now_us, const std::function<void(uint32_t now_ms)>& pump);

    Hub& hub() { return *_hub; }
    SimUiToken& minter() { return _minter; }
    bench::SessionLog log;

private:
    class GeigerToSessionLog final : public geiger::ISink {
    public:
        explicit GeigerToSessionLog(bench::SessionLog& log) : _log(log) {}
        void write(const geiger::Record& r) override;

    private:
        bench::SessionLog& _log;
    };
    // The board's ValenceLogBridge: Warn and above onto the log channel
    // 0x0008. Never logs: a GLOG from here would drain straight back in.
    class GeigerToLogChannel final : public geiger::ISink {
    public:
        void write(const geiger::Record& r) override;
        Hub* hub = nullptr;
    };

    uint64_t loadOrMintInstanceId();

    // Heap, not stack: the catalog pools and the hub's session table are tens
    // of KB, and the Hub sits in an optional so the catalog is FILLED before
    // the Hub constructor encodes it (the same order the P4's HubBox keeps).
    Catalog32 _catalog{};
    bench::HostRandom _rng{};
    ValenceDevice _device{};
    std::optional<Hub> _hub{};
    SimUiToken _minter{};
    std::array<std::byte, PatternPresetStore::kBlobBytes> _scratch{};
    ISimStore* _store = nullptr;
    const char* _storeName = "";
    uint32_t _lastHubMs = 0;
    GeigerToSessionLog _geigerSink{log};
    GeigerToLogChannel _logChannel{};
};

}  // namespace valence
