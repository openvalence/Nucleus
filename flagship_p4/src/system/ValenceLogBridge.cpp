// ValenceLogBridge -- implementation. See ValenceLogBridge.h for the task and
// floor contract.

#include "system/ValenceLogBridge.h"

#include <cstring>
#include <string_view>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "geiger/geiger.h"
#include "hub/ValenceHub.h"

namespace valence {

namespace {

// The hub task's name at its create site (hub/ValenceHub.cpp). If that name
// changes, this sink publishes nothing and says nothing: keep the two equal.
constexpr const char* kHubTaskName = "ValenceHub";

class LogBridge final : public geiger::ISink {
public:
    // Never logs: a GLOG from here would be drained straight back into it.
    void write(const geiger::Record& r) override {
        valence::Hub* h = hub();
        if (h == nullptr) return;
        const char* self = pcTaskGetName(nullptr);
        if (self == nullptr || std::strcmp(self, kHubTaskName) != 0) return;
        // geiger::Level and the registry's log_levels share one numbering.
        h->publishLog(uint8_t(r.level), std::string_view(r.tag), std::string_view(r.msg));
    }
};

LogBridge g_bridge;

}  // namespace

bool logBridgeBegin() {
    return geiger::logger().addSink(&g_bridge, geiger::Level::Warn);
}

}  // namespace valence
