// barnacle_update -- the helper's firmware update over the barnacle link: the
// P4's sender, the helper's receiver and the helper's trial, each an explicit
// state enum and transition table
// Constraints:
// - Header-only, hardware-free, allocation-free, no exceptions, as
//   barnacle_link.hpp. The P4, the shim and the native suite
//   (test_barnacle_update) compile this one file.
// - THE ONE HOME of the update layer's timing and its three machines. Its
//   frames, codes and chunk size are barnacle_link.hpp's, and FROZEN.
// - Each machine is a table of rules, (state, event) -> (next, action). A
//   cell with no rule keeps the state, does nothing and counts in ignored();
//   the suite checks every cell. SINGLE OWNER per instance: the P4's link
//   task, the helper's loop.
// - STOP AND WAIT. The P4 sends one update frame and waits for its ack. The
//   helper's UART interrupt is not in IRAM (the Arduino core's sdkconfig), so
//   while it erases or writes flash only the 128-byte RX FIFO takes what
//   arrives: nothing but a command frame may be in flight then.
// - THE IDLE RULE IS THE CALLER'S, never restated here: the P4's glue says
//   idle or busy every pass (BarnacleLink.h helperUpdateRefusal(): no motion
//   runs and no P4 update is in flight; motor power and the E-stop never gate
//   it). Busy stops a transfer with ABORT and the helper keeps the image it
//   runs.
// - AN ATTEMPT STARTS WITH start, never begin: the caller unhomes the machine
//   (MotionArbiter::unhome(), operator ruling 2026-11-01), then sends BEGIN.
//   The helper's stall sense is gone until it is back, so no home survives
//   the update. A resent BEGIN is begin and unhomes nothing.
// - NO PAIR PRESS: the P4 OTA that brought this image's copy of the shim
//   already had one (ValenceOta.h). Nothing here asks the operator.
// - THE ROLLBACK IS THE HELPER BOOTLOADER'S (CONFIG_BOOTLOADER_APP_ROLLBACK_
//   ENABLE in the core's bootloader). A new image boots on trial and keeps
//   itself only on a KEEP, which the P4 sends only once it has heard the new
//   image's ident: both directions proven. Without one inside kTrialMs, or at
//   any reset before it, the helper boots its previous image again.
// See: barnacle_link.hpp (the frames), flagship_p4/src/system/BarnacleLink.cpp
// (the P4's glue), barnacle_shim/src/main.cpp (the helper's), bd val-omh3.7

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "barnacle_link/barnacle_link.hpp"

namespace barnacle {

// ---- timing: THE KNOBS ------------------------------------------------------

// An update frame unanswered this long is sent again. A chunk is 22 ms on the
// wire; the helper may then erase a 4 KB sector, assumed under 500 ms (SPI
// NOR datasheet worst cases), during which it reads nothing.
inline constexpr uint32_t kAckTimeoutMs = 1000;
// END makes the helper read back and hash the whole image, then write its
// boot selection.
inline constexpr uint32_t kEndTimeoutMs = 3000;
// Sends of one frame before the attempt fails.
inline constexpr uint32_t kMaxTries = 5;
// From the helper's "verified" to the new image's first ident, and the
// longest silence a helper on trial may keep.
inline constexpr uint32_t kRestartMs = 10000;
// From the helper's boot on trial to its KEEP: past it, the helper reverts.
inline constexpr uint32_t kTrialMs = 30000;
// An open update with no update frame this long is dropped by the helper.
inline constexpr uint32_t kReceiverIdleMs = 10000;
// After a failed attempt, the next waits this long; kMaxAttempts per P4 boot.
inline constexpr uint32_t kRetryBackoffMs = 10000;
inline constexpr uint32_t kMaxAttempts = 3;
// Slow frames with no ident between them: a shim before v5, which has no
// update layer.
inline constexpr uint32_t kLegacySlowFrames = 5;
// The P4's command period while a transfer owns the link, so the chunks get
// the wire: the helper holds the last command it was given.
inline constexpr uint32_t kUpdateCommandMs = 50;

static_assert(kReceiverIdleMs > kAckTimeoutMs * kMaxTries, "the helper outwaits the P4's resends");
static_assert(kUpdateCommandMs * 4 <= kP4SilentMs, "the helper never reads the P4 silent during a transfer");
static_assert(kTrialMs > kRestartMs, "the P4 gives up on a trial before the helper reverts it");

// ---- the table shape --------------------------------------------------------

template <typename S, typename A>
struct Cell {
    S to{};
    A act{};
    bool legal = false;
};

template <typename S, typename E, typename A>
struct Rule {
    S from;
    E ev;
    S to;
    A act;
};

template <size_t NS, size_t NE, typename S, typename E, typename A, size_t NR>
constexpr std::array<std::array<Cell<S, A>, NE>, NS> buildTable(const std::array<Rule<S, E, A>, NR>& rules) {
    std::array<std::array<Cell<S, A>, NE>, NS> t{};
    for (const Rule<S, E, A>& r : rules) t[size_t(r.from)][size_t(r.ev)] = Cell<S, A>{r.to, r.act, true};
    return t;
}

template <typename S, typename E, typename A, size_t NR>
constexpr bool rulesUnique(const std::array<Rule<S, E, A>, NR>& rules) {
    for (size_t i = 0; i < NR; ++i)
        for (size_t j = i + 1; j < NR; ++j)
            if (rules[i].from == rules[j].from && rules[i].ev == rules[j].ev) return false;
    return true;
}

// One event applied: what the caller logs.
template <typename S, typename E, typename A>
struct Step {
    S from{};
    E ev{};
    S to{};
    A act{};
    bool ignored = false;   // no rule for the cell
    bool changed() const { return from != to; }
};

// ---- the P4's sender --------------------------------------------------------

enum class SendState : uint8_t {
    listen,       // no ident from the helper yet
    legacy,       // slow frames and no ident: a shim before v5, never updated over the link
    current,      // the helper runs this firmware's copy
    waiting,      // the helper runs another image: the transfer waits for idle (no motion)
    held,         // the helper runs another image ON TRIAL: it is kept or reverts first
    beginning,    // BEGIN sent, its ack awaited
    sending,      // a chunk sent, its ack awaited
    ending,       // END sent, its ack awaited
    restarting,   // the helper verified the image and restarts on it
    keeping,      // the helper runs this copy on trial: KEEP on every ident until it holds
    failed,       // the last attempt failed: the next waits kRetryBackoffMs
    gave_up,      // kMaxAttempts failed this boot
};
inline constexpr size_t kSendStates = 12;

enum class SendEvent : uint8_t {
    ident_ours,          // an ident: this copy, kept
    ident_ours_trial,    // an ident: this copy, on trial
    ident_other,         // an ident: another image, kept
    ident_other_trial,   // an ident: another image, on trial
    legacy_seen,         // kLegacySlowFrames slow frames without an ident
    idle,                // a pass while the machine is idle and the helper fresh
    busy,                // a pass while the machine is not idle
    ack_ok,              // the helper wants the chunk the ack names
    ack_all,             // the helper has every chunk
    ack_stale,           // an ack for a chunk before the one in flight
    ack_done,            // the image verified, set to boot
    ack_error,           // any refusal (lastCode())
    retry,               // the frame in flight went unanswered, sends left
    expire,              // no sends left, or a restart or trial went silent
    backoff_over,        // the next attempt may start
    give_up,             // the backoff ended with kMaxAttempts spent
};
inline constexpr size_t kSendEvents = 16;

enum class SendAct : uint8_t {
    none,
    start,        // the caller unhomes the machine, then BEGIN: an attempt's first
    begin,        // send BEGIN again
    chunk,        // send the chunk next() names
    end,          // send END
    abort,        // send ABORT: the helper keeps the image it runs
    fail,         // count the attempt, start the backoff
    fail_abort,   // fail, and send ABORT
    await,        // start the restart deadline
    keep,         // send KEEP
    done,         // the helper kept this copy
};

constexpr const char* sendStateName(SendState s) {
    switch (s) {
        case SendState::listen:     return "listen";
        case SendState::legacy:     return "legacy";
        case SendState::current:    return "current";
        case SendState::waiting:    return "waiting";
        case SendState::held:       return "held";
        case SendState::beginning:  return "beginning";
        case SendState::sending:    return "sending";
        case SendState::ending:     return "ending";
        case SendState::restarting: return "restarting";
        case SendState::keeping:    return "keeping";
        case SendState::failed:     return "failed";
        case SendState::gave_up:    return "gave-up";
    }
    return "?";
}

constexpr const char* sendEventName(SendEvent e) {
    switch (e) {
        case SendEvent::ident_ours:        return "ident-ours";
        case SendEvent::ident_ours_trial:  return "ident-ours-trial";
        case SendEvent::ident_other:       return "ident-other";
        case SendEvent::ident_other_trial: return "ident-other-trial";
        case SendEvent::legacy_seen:       return "legacy";
        case SendEvent::idle:              return "idle";
        case SendEvent::busy:              return "busy";
        case SendEvent::ack_ok:            return "ack-ok";
        case SendEvent::ack_all:           return "ack-all";
        case SendEvent::ack_stale:         return "ack-stale";
        case SendEvent::ack_done:          return "ack-done";
        case SendEvent::ack_error:         return "ack-error";
        case SendEvent::retry:             return "retry";
        case SendEvent::expire:            return "expire";
        case SendEvent::backoff_over:      return "backoff-over";
        case SendEvent::give_up:           return "give-up";
    }
    return "?";
}

namespace send {
using S = SendState;
using E = SendEvent;
using A = SendAct;
using R = Rule<S, E, A>;

inline constexpr std::array kRules = std::to_array<R>({
    // At rest, an ident says where the helper stands.
    R{S::listen, E::ident_ours, S::current, A::none},
    R{S::listen, E::ident_ours_trial, S::keeping, A::keep},
    R{S::listen, E::ident_other, S::waiting, A::none},
    R{S::listen, E::ident_other_trial, S::held, A::none},
    R{S::listen, E::legacy_seen, S::legacy, A::none},
    R{S::listen, E::idle, S::listen, A::none},
    R{S::listen, E::busy, S::listen, A::none},
    R{S::legacy, E::ident_ours, S::current, A::none},
    R{S::legacy, E::ident_ours_trial, S::keeping, A::keep},
    R{S::legacy, E::ident_other, S::waiting, A::none},
    R{S::legacy, E::ident_other_trial, S::held, A::none},
    R{S::legacy, E::legacy_seen, S::legacy, A::none},
    R{S::legacy, E::idle, S::legacy, A::none},
    R{S::legacy, E::busy, S::legacy, A::none},
    R{S::current, E::ident_ours, S::current, A::none},
    R{S::current, E::ident_ours_trial, S::keeping, A::keep},
    R{S::current, E::ident_other, S::waiting, A::none},
    R{S::current, E::ident_other_trial, S::held, A::none},
    R{S::current, E::legacy_seen, S::legacy, A::none},
    R{S::current, E::idle, S::current, A::none},
    R{S::current, E::busy, S::current, A::none},
    R{S::held, E::ident_ours, S::current, A::none},
    R{S::held, E::ident_ours_trial, S::keeping, A::keep},
    R{S::held, E::ident_other, S::waiting, A::none},
    R{S::held, E::ident_other_trial, S::held, A::none},
    R{S::held, E::legacy_seen, S::legacy, A::none},
    R{S::held, E::idle, S::held, A::none},
    R{S::held, E::busy, S::held, A::none},
    // waiting: its idents as anywhere at rest, but idle starts the transfer.
    R{S::waiting, E::ident_ours, S::current, A::none},
    R{S::waiting, E::ident_ours_trial, S::keeping, A::keep},
    R{S::waiting, E::ident_other, S::waiting, A::none},
    R{S::waiting, E::ident_other_trial, S::held, A::none},
    R{S::waiting, E::legacy_seen, S::legacy, A::none},
    R{S::waiting, E::idle, S::beginning, A::start},
    R{S::waiting, E::busy, S::waiting, A::none},
    // beginning: mid-transfer an ident changes nothing, the acks carry it.
    R{S::beginning, E::ident_ours, S::beginning, A::none},
    R{S::beginning, E::ident_ours_trial, S::beginning, A::none},
    R{S::beginning, E::ident_other, S::beginning, A::none},
    R{S::beginning, E::ident_other_trial, S::beginning, A::none},
    R{S::beginning, E::idle, S::beginning, A::none},
    R{S::beginning, E::ack_ok, S::sending, A::chunk},
    R{S::beginning, E::ack_error, S::failed, A::fail},
    R{S::beginning, E::retry, S::beginning, A::begin},
    R{S::beginning, E::expire, S::failed, A::fail_abort},
    R{S::beginning, E::busy, S::waiting, A::abort},
    // sending
    R{S::sending, E::ident_ours, S::sending, A::none},
    R{S::sending, E::ident_ours_trial, S::sending, A::none},
    R{S::sending, E::ident_other, S::sending, A::none},
    R{S::sending, E::ident_other_trial, S::sending, A::none},
    R{S::sending, E::idle, S::sending, A::none},
    R{S::sending, E::ack_ok, S::sending, A::chunk},
    R{S::sending, E::ack_all, S::ending, A::end},
    R{S::sending, E::ack_stale, S::sending, A::none},
    R{S::sending, E::ack_error, S::failed, A::fail},
    R{S::sending, E::retry, S::sending, A::chunk},
    R{S::sending, E::expire, S::failed, A::fail_abort},
    R{S::sending, E::busy, S::waiting, A::abort},
    // ending: a late ack for a chunk changes nothing. This copy's ident is
    // the helper back on the new image: its "verified" was lost.
    R{S::ending, E::ident_ours, S::current, A::done},
    R{S::ending, E::ident_ours_trial, S::keeping, A::keep},
    R{S::ending, E::ident_other, S::ending, A::none},
    R{S::ending, E::ident_other_trial, S::ending, A::none},
    R{S::ending, E::idle, S::ending, A::none},
    R{S::ending, E::ack_ok, S::ending, A::none},
    R{S::ending, E::ack_all, S::ending, A::none},
    R{S::ending, E::ack_stale, S::ending, A::none},
    R{S::ending, E::ack_done, S::restarting, A::await},
    R{S::ending, E::ack_error, S::failed, A::fail},
    R{S::ending, E::retry, S::ending, A::end},
    R{S::ending, E::expire, S::failed, A::fail_abort},
    R{S::ending, E::busy, S::waiting, A::abort},
    // restarting: the helper's flash work is done; it restarts whatever the
    // machine does. Back on another image is its bootloader refusing ours.
    R{S::restarting, E::ident_ours, S::current, A::done},
    R{S::restarting, E::ident_ours_trial, S::keeping, A::keep},
    R{S::restarting, E::ident_other, S::failed, A::fail},
    R{S::restarting, E::ident_other_trial, S::failed, A::fail},
    R{S::restarting, E::ack_done, S::restarting, A::none},   // a resent END's answer
    R{S::restarting, E::expire, S::failed, A::fail},
    R{S::restarting, E::idle, S::restarting, A::none},
    R{S::restarting, E::busy, S::restarting, A::none},
    // keeping: another image now is the trial reverting.
    R{S::keeping, E::ident_ours, S::current, A::done},
    R{S::keeping, E::ident_ours_trial, S::keeping, A::keep},
    R{S::keeping, E::ident_other, S::failed, A::fail},
    R{S::keeping, E::ident_other_trial, S::failed, A::fail},
    R{S::keeping, E::expire, S::failed, A::fail},
    R{S::keeping, E::idle, S::keeping, A::none},
    R{S::keeping, E::busy, S::keeping, A::none},
    // failed: the backoff runs; the helper may meanwhile turn out current.
    R{S::failed, E::ident_ours, S::current, A::none},
    R{S::failed, E::ident_ours_trial, S::keeping, A::keep},
    R{S::failed, E::ident_other, S::failed, A::none},
    R{S::failed, E::ident_other_trial, S::failed, A::none},
    R{S::failed, E::legacy_seen, S::legacy, A::none},
    R{S::failed, E::backoff_over, S::waiting, A::none},
    R{S::failed, E::give_up, S::gave_up, A::none},
    R{S::failed, E::idle, S::failed, A::none},
    R{S::failed, E::busy, S::failed, A::none},
    // gave_up: only a helper that turns out current leaves it.
    R{S::gave_up, E::ident_ours, S::current, A::none},
    R{S::gave_up, E::ident_ours_trial, S::keeping, A::keep},
    R{S::gave_up, E::ident_other, S::gave_up, A::none},
    R{S::gave_up, E::ident_other_trial, S::gave_up, A::none},
    R{S::gave_up, E::legacy_seen, S::legacy, A::none},
    R{S::gave_up, E::idle, S::gave_up, A::none},
    R{S::gave_up, E::busy, S::gave_up, A::none},
});

static_assert(rulesUnique(kRules), "one rule per cell");
inline constexpr auto kTable = buildTable<kSendStates, kSendEvents>(kRules);
}  // namespace send

constexpr Cell<SendState, SendAct> sendCell(SendState s, SendEvent e) {
    return send::kTable[size_t(s)][size_t(e)];
}

using SendStep = Step<SendState, SendEvent, SendAct>;

// The P4's half. Owns no I/O: each call returns the step it took, and
// frameFor() encodes the frame that step's action sends.
class UpdateSender {
public:
    // `image` is the shim's .bin, its digest last. It must outlive the sender.
    explicit UpdateSender(std::span<const uint8_t> image) : _image(image) {
        _valid = image.size() > kDigestBytes && image.size() <= kMaxImageBytes;
        if (!_valid) return;
        for (size_t k = 0; k < kDigestBytes; ++k) _digest[k] = image[image.size() - kDigestBytes + k];
        _chunks = chunkCount(uint32_t(image.size()));
    }

    // An image too small to carry its digest or too large for the index: the
    // caller never runs the sender.
    bool valid() const { return _valid; }

    SendStep onIdent(const Ident& d, uint32_t nowMs) {
        _helper = d;
        _haveHelper = true;
        _slowSinceIdent = 0;
        const bool ours = ourDigest(d.digest);
        const bool trial = (d.flags & kIdentTrial) != 0;
        const SendEvent e = ours ? (trial ? SendEvent::ident_ours_trial : SendEvent::ident_ours)
                                 : (trial ? SendEvent::ident_other_trial : SendEvent::ident_other);
        return apply(e, nowMs);
    }

    // A slow frame: kLegacySlowFrames of them with no ident is a shim that
    // has no update layer. Nullopt while that is not yet the case.
    std::optional<SendStep> onSlow(uint32_t nowMs) {
        if (++_slowSinceIdent < kLegacySlowFrames) return std::nullopt;
        _slowSinceIdent = 0;
        return apply(SendEvent::legacy_seen, nowMs);
    }

    SendStep onAck(const UpdateAck& a, uint32_t nowMs) {
        SendEvent e = SendEvent::ack_error;
        if (a.code == AckCode::done) {
            e = SendEvent::ack_done;
        } else if (a.code != AckCode::ok) {
            _lastCode = a.code;
        } else if (a.next > _chunks) {
            _lastCode = AckCode::malformed;
        } else if (a.next < _next) {
            e = SendEvent::ack_stale;
        } else {
            e = a.next == _chunks ? SendEvent::ack_all : SendEvent::ack_ok;
            if (_state == SendState::beginning || _state == SendState::sending) {
                _next = a.next;
                _tries = 0;
            }
        }
        return apply(e, nowMs);
    }

    // Every pass. `idle`: the caller's idle rule passes; `fresh`: the helper
    // is heard. A deadline due is the step; else idle or busy, and nothing
    // while idle with the helper unheard.
    std::optional<SendStep> tick(uint32_t nowMs, bool idle, bool fresh) {
        if (_armed && int32_t(nowMs - _deadlineMs) >= 0) {
            _armed = false;
            switch (_state) {
                case SendState::beginning:
                case SendState::sending:
                case SendState::ending:
                    return apply(_tries < kMaxTries ? SendEvent::retry : SendEvent::expire, nowMs);
                case SendState::restarting:
                case SendState::keeping:
                    return apply(SendEvent::expire, nowMs);
                case SendState::failed:
                    return apply(_attempts >= kMaxAttempts ? SendEvent::give_up : SendEvent::backoff_over, nowMs);
                default:
                    break;
            }
        }
        if (!idle) return apply(SendEvent::busy, nowMs);
        if (fresh) return apply(SendEvent::idle, nowMs);
        return std::nullopt;
    }

    // The frame `act` sends, encoded into `out`; 0 for an action that sends
    // nothing.
    size_t frameFor(SendAct act, uint8_t seq, std::span<uint8_t> out) const {
        switch (act) {
            case SendAct::start:
            case SendAct::begin: {
                UpdateBegin b;
                b.size = uint32_t(_image.size());
                b.digest = _digest;
                return encode(b, seq, out);
            }
            case SendAct::chunk: {
                const size_t at = size_t(_next) * kChunkBytes;
                if (at >= _image.size()) return 0;
                const size_t n = _image.size() - at < kChunkBytes ? _image.size() - at : kChunkBytes;
                UpdateData d;
                d.index = uint16_t(_next);
                d.bytes = _image.subspan(at, n);
                return encode(d, seq, out);
            }
            case SendAct::end:        return encodeBare(Type::update_end, seq, out);
            case SendAct::abort:
            case SendAct::fail_abort: return encodeBare(Type::update_abort, seq, out);
            case SendAct::keep: {
                UpdateKeep k;
                for (size_t i = 0; i < kIdentDigestBytes; ++i) k.digest[i] = _digest[i];
                return encode(k, seq, out);
            }
            default:                  return 0;
        }
    }

    SendState state() const { return _state; }
    bool transferring() const {
        return _state == SendState::beginning || _state == SendState::sending || _state == SendState::ending;
    }
    // From BEGIN to the helper's restart: the link carries the update.
    bool updating() const { return transferring() || _state == SendState::restarting; }
    uint32_t next() const { return _next; }
    uint32_t chunks() const { return _chunks; }
    uint32_t tries() const { return _tries; }
    uint32_t attempts() const { return _attempts; }
    AckCode lastCode() const { return _lastCode; }
    bool haveHelper() const { return _haveHelper; }
    const Ident& helper() const { return _helper; }
    const Digest& digest() const { return _digest; }
    size_t imageBytes() const { return _image.size(); }
    // The first BEGIN of the attempt that is running or last ran; nullopt
    // when this boot sent none.
    std::optional<uint32_t> startedMs() const {
        return _started ? std::optional<uint32_t>(_startedMs) : std::nullopt;
    }
    uint32_t ignored() const { return _ignored; }

private:
    bool ourDigest(const IdentDigest& d) const {
        for (size_t k = 0; k < kIdentDigestBytes; ++k)
            if (d[k] != _digest[k]) return false;
        return true;
    }

    void arm(uint32_t nowMs, uint32_t afterMs) {
        _deadlineMs = nowMs + afterMs;
        _armed = true;
    }

    SendStep apply(SendEvent e, uint32_t nowMs) {
        const Cell<SendState, SendAct> c = sendCell(_state, e);
        SendStep s;
        s.from = _state;
        s.ev = e;
        if (!c.legal) {
            ++_ignored;
            s.to = _state;
            s.ignored = true;
            return s;
        }
        s.to = c.to;
        s.act = c.act;
        if (c.to != _state) _armed = false;
        _state = c.to;
        switch (c.act) {
            case SendAct::start:
                _started = true;
                _startedMs = nowMs;
                _next = 0;
                _tries = 0;
                [[fallthrough]];
            case SendAct::begin:
                ++_tries;
                arm(nowMs, kAckTimeoutMs);
                break;
            case SendAct::chunk:
                ++_tries;
                arm(nowMs, kAckTimeoutMs);
                break;
            case SendAct::end:
                if (s.from != SendState::ending) _tries = 0;
                ++_tries;
                arm(nowMs, kEndTimeoutMs);
                break;
            case SendAct::fail:
            case SendAct::fail_abort:
                ++_attempts;
                arm(nowMs, kRetryBackoffMs);
                break;
            case SendAct::await:
            case SendAct::keep:
                arm(nowMs, kRestartMs);
                break;
            default:
                break;
        }
        return s;
    }

    std::span<const uint8_t> _image;
    bool _valid = false;
    Digest _digest{};
    uint32_t _chunks = 0;
    SendState _state = SendState::listen;
    uint32_t _next = 0;
    uint32_t _tries = 0;
    uint32_t _attempts = 0;
    bool _armed = false;
    uint32_t _deadlineMs = 0;
    bool _started = false;
    uint32_t _startedMs = 0;
    AckCode _lastCode = AckCode::ok;
    Ident _helper{};
    bool _haveHelper = false;
    uint32_t _slowSinceIdent = 0;
    uint32_t _ignored = 0;
};

// ---- the helper's receiver --------------------------------------------------

// Where the image goes: the helper's spare app slot (esp_ota_*). The sink
// hashes nothing itself; finish() is esp_ota_end(), which checks the image in
// flash against its own appended digest, then sets it to boot.
class UpdateSink {
public:
    virtual uint32_t capacity() const = 0;   // the spare slot's bytes
    virtual bool begin(uint32_t size) = 0;   // open the spare slot for `size` bytes
    virtual bool write(std::span<const uint8_t> bytes) = 0;   // the next bytes, in order
    virtual bool finish() = 0;               // verify, set to boot; closes the slot either way
    virtual void abort() = 0;                // close the slot, keep nothing

protected:
    ~UpdateSink() = default;
};

enum class RecvState : uint8_t {
    idle,        // no update open
    receiving,   // an update open: chunks expected in order
    done,        // the image verified and set to boot: the caller restarts
};
inline constexpr size_t kRecvStates = 3;

enum class RecvEvent : uint8_t {
    begin,           // a BEGIN this helper can take
    begin_refused,   // a BEGIN it cannot: on trial, too big, malformed
    chunk,           // the chunk wanted
    chunk_again,     // a well-formed chunk that is not the one wanted
    chunk_bad,       // a chunk of the wrong length, or past the image
    end,             // END with every byte in and the digest matching
    end_short,       // END before every byte
    end_digest,      // END, the image's tail not the digest begun
    abort,
    timeout,         // kReceiverIdleMs without an update frame
};
inline constexpr size_t kRecvEvents = 10;

enum class RecvAct : uint8_t {
    none,
    open,           // sink.begin(), ack ok next 0
    reopen,         // sink.abort(), then open
    refuse,         // ack the refusal, nothing open
    no_session,     // ack no_session
    store,          // sink.write(), ack ok next
    reack,          // ack ok next, nothing written
    close,          // sink.abort(), no ack
    close_refuse,   // sink.abort(), ack the refusal
    finish,         // sink.finish(), ack done
};

namespace recv {
using S = RecvState;
using E = RecvEvent;
using A = RecvAct;
using R = Rule<S, E, A>;

inline constexpr std::array kRules = std::to_array<R>({
    R{S::idle, E::begin, S::receiving, A::open},
    R{S::idle, E::begin_refused, S::idle, A::refuse},
    R{S::idle, E::chunk, S::idle, A::no_session},
    R{S::idle, E::chunk_again, S::idle, A::no_session},
    R{S::idle, E::chunk_bad, S::idle, A::no_session},
    R{S::idle, E::end, S::idle, A::no_session},
    R{S::idle, E::end_short, S::idle, A::no_session},
    R{S::idle, E::end_digest, S::idle, A::no_session},
    R{S::idle, E::abort, S::idle, A::none},
    R{S::receiving, E::begin, S::receiving, A::reopen},
    R{S::receiving, E::begin_refused, S::idle, A::close_refuse},
    R{S::receiving, E::chunk, S::receiving, A::store},
    R{S::receiving, E::chunk_again, S::receiving, A::reack},
    R{S::receiving, E::chunk_bad, S::idle, A::close_refuse},
    R{S::receiving, E::end, S::done, A::finish},
    R{S::receiving, E::end_short, S::idle, A::close_refuse},
    R{S::receiving, E::end_digest, S::idle, A::close_refuse},
    R{S::receiving, E::abort, S::idle, A::close},
    R{S::receiving, E::timeout, S::idle, A::close},
    // done: the caller is restarting the chip; nothing more is taken.
    R{S::done, E::begin, S::done, A::none},
    R{S::done, E::begin_refused, S::done, A::none},
    R{S::done, E::chunk, S::done, A::none},
    R{S::done, E::chunk_again, S::done, A::none},
    R{S::done, E::chunk_bad, S::done, A::none},
    R{S::done, E::end, S::done, A::none},
    R{S::done, E::end_short, S::done, A::none},
    R{S::done, E::end_digest, S::done, A::none},
    R{S::done, E::abort, S::done, A::none},
});
static_assert(rulesUnique(kRules), "one rule per cell");
inline constexpr auto kTable = buildTable<kRecvStates, kRecvEvents>(kRules);
}  // namespace recv

constexpr Cell<RecvState, RecvAct> recvCell(RecvState s, RecvEvent e) {
    return recv::kTable[size_t(s)][size_t(e)];
}

using RecvStep = Step<RecvState, RecvEvent, RecvAct>;

// The helper's half. A sink action that fails closes the update and answers
// with the failure, whatever the cell's next state: the one exception to
// the table, and the suite drives each.
class UpdateReceiver {
public:
    explicit UpdateReceiver(UpdateSink& sink) : _sink(sink) {}

    // One verified frame. Frames outside the update layer, and KEEP (the
    // trial's), are not this machine's: nullopt, nothing recorded. Else the
    // ack to send, if the action sends one.
    std::optional<UpdateAck> onFrame(const Frame& f, uint32_t nowMs, bool onTrial) {
        if (f.version != kUpdateVersion) return std::nullopt;
        RecvEvent e;
        std::optional<UpdateData> data;
        switch (f.type) {
            case Type::update_begin: {
                const std::optional<UpdateBegin> b = asUpdateBegin(f);
                if (!b) {
                    e = RecvEvent::begin_refused;
                    _why = AckCode::malformed;
                    break;
                }
                e = classifyBegin(*b, onTrial);
                if (e == RecvEvent::begin) _pending = *b;
                break;
            }
            case Type::update_data:
                data = asUpdateData(f);
                e = data ? classifyChunk(*data) : RecvEvent::chunk_bad;
                if (!data) _why = AckCode::malformed;
                break;
            case Type::update_end:
                e = classifyEnd();
                break;
            case Type::update_abort:
                e = RecvEvent::abort;
                break;
            default:
                return std::nullopt;
        }
        if (_state == RecvState::receiving) _lastMs = nowMs;
        return apply(e, nowMs, data);
    }

    void tick(uint32_t nowMs) {
        if (_state == RecvState::receiving && nowMs - _lastMs >= kReceiverIdleMs)
            apply(RecvEvent::timeout, nowMs, std::nullopt);
    }

    RecvState state() const { return _state; }
    const RecvStep& last() const { return _last; }
    uint32_t next() const { return _next; }
    uint32_t received() const { return _received; }
    uint32_t ignored() const { return _ignored; }

private:
    RecvEvent classifyBegin(const UpdateBegin& b, bool onTrial) {
        if (onTrial) {
            _why = AckCode::on_trial;
            return RecvEvent::begin_refused;
        }
        if (b.size <= kDigestBytes) {
            _why = AckCode::malformed;
            return RecvEvent::begin_refused;
        }
        if (b.size > kMaxImageBytes || b.size > _sink.capacity()) {
            _why = AckCode::too_big;
            return RecvEvent::begin_refused;
        }
        return RecvEvent::begin;
    }

    RecvEvent classifyChunk(const UpdateData& d) {
        _why = AckCode::malformed;
        if (_state != RecvState::receiving) return RecvEvent::chunk;   // no_session whatever it is
        if (d.index >= _chunks) return RecvEvent::chunk_bad;
        const uint32_t at = uint32_t(d.index) * uint32_t(kChunkBytes);
        const uint32_t want = _open.size - at < kChunkBytes ? _open.size - at : uint32_t(kChunkBytes);
        if (d.bytes.size() != want) return RecvEvent::chunk_bad;
        return d.index == _next ? RecvEvent::chunk : RecvEvent::chunk_again;
    }

    RecvEvent classifyEnd() {
        if (_state != RecvState::receiving) return RecvEvent::end;   // no_session
        if (_received < _open.size) {
            _why = AckCode::short_image;
            return RecvEvent::end_short;
        }
        if (_tail != _open.digest) {
            _why = AckCode::digest;
            return RecvEvent::end_digest;
        }
        return RecvEvent::end;
    }

    UpdateAck ack(AckCode c) const {
        UpdateAck a;
        a.code = c;
        a.next = uint16_t(_next);
        return a;
    }

    void openSession() {
        _open = _pending;
        _chunks = chunkCount(_open.size);
        _next = 0;
        _received = 0;
        _tail = {};
    }

    void keepTail(std::span<const uint8_t> bytes) {
        const uint32_t tailAt = _open.size - uint32_t(kDigestBytes);
        for (size_t k = 0; k < bytes.size(); ++k) {
            const uint32_t at = _received + uint32_t(k);
            if (at >= tailAt) _tail[at - tailAt] = bytes[k];
        }
    }

    std::optional<UpdateAck> apply(RecvEvent e, uint32_t nowMs, const std::optional<UpdateData>& data) {
        const Cell<RecvState, RecvAct> c = recvCell(_state, e);
        _last = RecvStep{};
        _last.from = _state;
        _last.ev = e;
        if (!c.legal) {
            ++_ignored;
            _last.to = _state;
            _last.ignored = true;
            return std::nullopt;
        }
        _last.to = c.to;
        _last.act = c.act;
        _state = c.to;
        std::optional<UpdateAck> out;
        switch (c.act) {
            case RecvAct::none:
                break;
            case RecvAct::reopen:
                _sink.abort();
                [[fallthrough]];
            case RecvAct::open:
                openSession();
                _lastMs = nowMs;
                if (!_sink.begin(_open.size)) return failed(AckCode::flash);
                out = ack(AckCode::ok);
                break;
            case RecvAct::refuse:
                out = ack(_why);
                break;
            case RecvAct::no_session:
                out = ack(AckCode::no_session);
                break;
            case RecvAct::store:
                if (!_sink.write(data->bytes)) {
                    _sink.abort();
                    return failed(AckCode::flash);
                }
                keepTail(data->bytes);
                _received += uint32_t(data->bytes.size());
                ++_next;
                out = ack(AckCode::ok);
                break;
            case RecvAct::reack:
                out = ack(AckCode::ok);
                break;
            case RecvAct::close:
                _sink.abort();
                break;
            case RecvAct::close_refuse:
                _sink.abort();
                out = ack(_why);
                break;
            case RecvAct::finish:
                if (!_sink.finish()) return failed(AckCode::image);
                out = ack(AckCode::done);
                break;
        }
        return out;
    }

    // The exception: a sink failure closes the update.
    std::optional<UpdateAck> failed(AckCode c) {
        _state = RecvState::idle;
        _last.to = RecvState::idle;
        return ack(c);
    }

    UpdateSink& _sink;
    RecvState _state = RecvState::idle;
    UpdateBegin _pending{};
    UpdateBegin _open{};
    uint32_t _chunks = 0;
    uint32_t _next = 0;
    uint32_t _received = 0;
    Digest _tail{};
    AckCode _why = AckCode::ok;
    uint32_t _lastMs = 0;
    RecvStep _last{};
    uint32_t _ignored = 0;
};

// ---- the helper's trial -----------------------------------------------------

enum class TrialState : uint8_t {
    none,        // the running image is kept (flashed over USB, or kept earlier)
    trial,       // the running image is on trial: a KEEP keeps it, kTrialMs reverts it
    kept,        // kept this boot
    reverting,   // the deadline passed: the caller reverts and restarts
};
inline constexpr size_t kTrialStates = 4;

enum class TrialEvent : uint8_t {
    keep_ours,    // a KEEP naming the running image
    keep_other,   // a KEEP naming another image
    deadline,     // kTrialMs since boot
};
inline constexpr size_t kTrialEvents = 3;

enum class TrialAct : uint8_t {
    none,
    mark_valid,   // esp_ota_mark_app_valid_cancel_rollback()
    revert,       // esp_ota_mark_app_invalid_rollback_and_reboot()
};

namespace trial {
using S = TrialState;
using E = TrialEvent;
using A = TrialAct;
using R = Rule<S, E, A>;

inline constexpr std::array kRules = std::to_array<R>({
    R{S::none, E::keep_ours, S::none, A::none},
    R{S::none, E::keep_other, S::none, A::none},
    R{S::trial, E::keep_ours, S::kept, A::mark_valid},
    R{S::trial, E::keep_other, S::trial, A::none},
    R{S::trial, E::deadline, S::reverting, A::revert},
    R{S::kept, E::keep_ours, S::kept, A::none},
    R{S::kept, E::keep_other, S::kept, A::none},
    R{S::reverting, E::keep_ours, S::reverting, A::none},
    R{S::reverting, E::keep_other, S::reverting, A::none},
});
static_assert(rulesUnique(kRules), "one rule per cell");
inline constexpr auto kTable = buildTable<kTrialStates, kTrialEvents>(kRules);
}  // namespace trial

constexpr Cell<TrialState, TrialAct> trialCell(TrialState s, TrialEvent e) {
    return trial::kTable[size_t(s)][size_t(e)];
}

using TrialStep = Step<TrialState, TrialEvent, TrialAct>;

class TrialGate {
public:
    TrialGate(bool onTrial, const IdentDigest& mine)
        : _state(onTrial ? TrialState::trial : TrialState::none), _mine(mine) {}

    TrialStep onKeep(const UpdateKeep& k) {
        return apply(k.digest == _mine ? TrialEvent::keep_ours : TrialEvent::keep_other);
    }

    // nowMs since the helper's boot. A step only when the deadline fires.
    std::optional<TrialStep> tick(uint32_t nowMs) {
        if (_state != TrialState::trial || nowMs < kTrialMs) return std::nullopt;
        return apply(TrialEvent::deadline);
    }

    TrialState state() const { return _state; }
    // What the ident's kIdentTrial flag says: not yet kept.
    bool onTrial() const { return _state == TrialState::trial || _state == TrialState::reverting; }

private:
    TrialStep apply(TrialEvent e) {
        const Cell<TrialState, TrialAct> c = trialCell(_state, e);
        TrialStep s;
        s.from = _state;
        s.ev = e;
        s.to = c.legal ? c.to : _state;
        s.act = c.legal ? c.act : TrialAct::none;
        s.ignored = !c.legal;
        _state = s.to;
        return s;
    }

    TrialState _state;
    IdentDigest _mine;
};

}  // namespace barnacle
