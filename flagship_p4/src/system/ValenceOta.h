#pragma once

// ValenceOta -- POST /ota on :80: raw-body firmware update into the idle app
// slot, with bootloader rollback as the undo
// Constraints:
// - RAW BODY, NEVER MULTIPART. The handler streams straight into
//   esp_ota_write; nothing here parses a form. curl --data-binary @image.bin
//   plus -H "Expect:" (esp_http_server never answers 100-continue).
// - AUTH IS THE X-OTA-Token HEADER, compared CONSTANT-TIME against
//   SECRET_OTA_TOKEN in the git-ignored secrets.h. It is the only check on the
//   path: a LAN host holding it can overwrite this board's firmware.
// - ONE MOTION GATE, NOT A SECOND LATCH. The handler calls motionEstop()
//   before the first write; the arbiter's existing latch is the gate. Motion
//   stays parked afterward whether the transfer succeeded or not, and a failed
//   update leaves the operator re-homing -- deliberate, because the safe state
//   after an aborted flash is not "carry on".
// - THE TRANSPORT CARRYING THE BYTES IS NEVER SUSPENDED (T31). The hub task
//   and the :82 WS port keep running for the whole transfer; the bytes arrive
//   on the :80 task, which is the one that blocks. PSRAM is unreachable while
//   the flash cache is off, and the hub box lives in PSRAM -- what makes that
//   safe is the IDF flash driver STALLING the other core for the duration of
//   each write, so the hub task is not running rather than running blind. No
//   application-level pause is needed and none is installed.
// - THE RUNNING SLOT IS NEVER TOUCHED. Writes go to the slot the bootloader is
//   not running from, so a short body or a wrong magic costs an unusable spare
//   slot and nothing else.
// - BUY LATE. CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE leaves a new image
//   PENDING_VERIFY; otaMarkAppValid() is the only call that commits it, and
//   the composition root makes it only once the hub is up and WiFi holds an
//   IP. A build that hangs before that resets on the hub task's watchdog and
//   the bootloader brings the previous slot back.
// See: .claude/rules/build-test-deploy.md, bd val-091.16

namespace valence {

// Registers POST /ota on the shared :80 instance. Non-fatal on failure: the
// machine runs, it just cannot be updated remotely.
bool otaBegin();

// Commits the running image, canceling the pending rollback. Idempotent and
// cheap after the first call. The caller is the composition root's liveness
// loop, which owns the "hub up and addressable" judgment.
void otaMarkAppValid();

// Boot-banner truth: the label of the partition actually running, and whether
// it is still on trial.
const char* otaRunningSlot();
bool otaPendingVerify();

}  // namespace valence
