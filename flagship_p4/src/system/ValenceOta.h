#pragma once

// ValenceOta -- the firmware update on :80: a request, a PAIR double press at
// the machine, then the raw image into the idle app slot, with bootloader
// rollback as the undo
// Constraints:
// - THE AUTHORITY IS PHYSICAL (operator ruling 2026-10-18, bd val-ktal). Any
//   LAN client may ask; only a PAIR double press on the machine inside the
//   request's window lets the image in, once. OTA never rides Valence (SPEC
//   1.2-8), so no Valence tier or token stands in for the press. The rules of
//   the request (one live, its window, one use) are OtaArm.h's.
//     POST /ota/arm          202 {"id":"<hex>","window_s":N}; 409 + reason
//     GET  /ota/arm?id=<hex> 200 {"state":"pending|approved|expired"}
//     POST /ota?id=<hex>     the raw image; 403 unless approved, 409 unless idle
// - A PENDING REQUEST OWNS THE DOUBLE PRESS. The hub task offers every PAIR
//   double press here first (otaPressTake()); one that approves a request
//   never also approves a Valence configure request. A press and a hold keep
//   their meanings.
// - IDLE OR NOTHING: arm and image are both refused 409 unless motion is
//   parked and motor power is off (OtaArm.h idleRefusal()). Nothing here
//   stops the machine. The release that would turn power back on is refused
//   while otaInFlight() (ValenceDevice::canClearEstop()).
// - THE BENCH IMAGE ALONE also takes X-OTA-Token (SECRET_OTA_TOKEN, the
//   git-ignored secrets.h) in place of the press; the idle gate still holds.
//   The release image has no key compiled in: one shared key in every
//   published .bin would open every machine.
// - RAW BODY, NEVER MULTIPART. The handler streams straight into
//   esp_ota_write; integrity is esp_ota's (image header, chip id and
//   revision, SHA-256). There is no signature check.
// - THE TRANSPORT CARRYING THE BYTES IS NEVER SUSPENDED (T31). The hub task
//   and the :82 WS port keep running for the whole transfer; the bytes arrive
//   on the :80 task, which is the one that blocks. PSRAM is unreachable while
//   the flash cache is off, and the hub box lives in PSRAM -- what makes that
//   safe is the IDF flash driver STALLING the other core for the duration of
//   each write, so the hub task is not running rather than running blind.
// - THE RUNNING SLOT IS NEVER TOUCHED. Writes go to the slot the bootloader is
//   not running from, so a short body or a wrong magic costs an unusable spare
//   slot and nothing else.
// - BUY LATE. CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE leaves a new image
//   PENDING_VERIFY; otaMarkAppValid() is the only call that commits it, and
//   the composition root makes it only once the hub is up and WiFi holds an
//   IP. A build that hangs before that resets on the hub task's watchdog and
//   the bootloader brings the previous slot back.
// - Declarations only, no IDF: the hardware-free hub delegate calls
//   otaPressTake() and otaInFlight(), so the host twins link their own.
// See: OtaArm.h, .claude/rules/build-test-deploy.md, bd val-091.16, val-ktal

namespace valence {

// Registers the three routes on the shared :80 instance. Non-fatal on
// failure: the machine runs, it just cannot be updated remotely.
bool otaBegin();

// Commits the running image, canceling the pending rollback. Idempotent and
// cheap after the first call. The caller is the composition root's liveness
// loop, which owns the "hub up and addressable" judgment.
void otaMarkAppValid();

// True from the moment an update is let in until the reboot that follows it
// (or until it fails). Any task; the hub task holds its NVS writes and the
// ESTOP release on it.
bool otaInFlight();

// Hub task, on a PAIR double press. True when it approved the pending update
// request: the press is spent and binds nothing else.
bool otaPressTake();

// Boot-banner truth: the label of the partition actually running, and whether
// it is still on trial.
const char* otaRunningSlot();
bool otaPendingVerify();

}  // namespace valence
