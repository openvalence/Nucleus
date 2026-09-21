---
paths:
  - "**"
---

# Transport and Valence boundary constraints

SPEC §13/§14 and the headers own the protocol story; this file is this
machine's half of the boundary plus the transport traps. Carried from the
archived machine repo (SlopDrive-32 `.claude/rules/transport.md`), minus the
UART bridge, which does not exist here.

## Valence (NON-NEGOTIABLE)

The ecosystem sync protocol is developed in its OWN repo, Valence (sibling
checkout `../Valence`, pinned at this repo's root by `valence.pin`). This
machine repo CONSUMES it, never edits it. The spec, registry, codegen, library
invariants, frozen-artifact list, layering and tests are that repo's doctrine;
restating them here would violate C-1. The protocol's rename to Valence landed
upstream with RFC-060 and is reflected here by following the pin; nothing
wire-visible is ever respelled FROM this repo (`governance.md` §6).

- **Consumption mechanics.** `lib/valence` is a real symlink to
  `../Valence/lib/valence`. The library ships no `.c` or `.cpp`, so an
  INCLUDE PATH is the whole of consuming it: no component, no build step. The
  path is registered in the project's own `idf_component_register`.
  `tools/canon_lint.py`'s pin rule FAILS if the sibling's HEAD does not match
  `valence.pin`, and cross-checks the sibling's frozen conformance artifacts
  against the same hashes Valence's own lint pins. Belt and suspenders across
  the repo boundary.
- **Spec-gap ritual, cross-repo order.** Need a number or rule the spec lacks:
  fix it in the Valence repo FIRST (`registry.yaml` or `SPEC.md`,
  regenerated, committed there), bump `valence.pin` to the new sha, THEN code
  against the constant here. Never a code-local magic number for anything
  wire-visible, and never a spec change made from this repo. Enforced by
  `.claude/hooks/vendor-lock.sh`, whose one writable door is the sibling's
  `spec/RFC-QUEUE.md`.
- **Frozen (C-6).** The conformance artifacts and the `hub.hpp`/`client.hpp`
  public API freeze are Valence's own frozen list, enforced by its lint. This
  repo's half is the sha256 cross-check in `tools/canon_lint.py`.
- **Firmware shape.** The hub service is the composition root, on its own
  task, single-task by design (T5), plus the transport ports plus this
  machine's catalog. A service with large state lives in PSRAM via
  placement-new from the composition root (`memory-budget.md` T2); never as
  BSS. Session teardown funnels through one path (T3), and **back-to-back
  sessions without a reboot is mandatory verification for any
  session-lifecycle change.**
- **Auth.** Token validation resolves the UI token, then the trust ledger,
  then `watch`. Tokenless clients can watch and e-stop (stop and estop are
  role-EXEMPT) but cannot command motion. While a UI token is enabled, LAN
  HTTP equals control; a lockdown posture buys a chokepoint, not LAN secrecy.
- **Valence is the ONLY input/output plane (operator ruling 2026-07-26).**
  Motion input, telemetry, anomaly events and settings ride Valence channels.
  HTTP remains for fallback polling and bootstrap only.
- **Transport doctrine (operator rulings 2026-07-27, calibrated).** Valence is
  the only protocol and is transport-agnostic (SPEC §13, RFC-043 profiles). For
  hardware hubs: BLE GATT is the conformance floor (infrastructure-free
  control, discovery, future WiFi provisioning); WebSocket is the preferred
  high-throughput path, expected on this class of silicon; clients auto-upgrade
  BLE to WS. ESP-NOW is the peer binding: supported and deliberately trivial
  to enable, NOT actively developed or tested. UI-serving is a hub capability,
  never a requirement. On this board BLE later means a NimBLE host on the P4
  with the controller on the C6 (`governance.md` §6).
- **Intake doctrine (operator ruling 2026-07-27).** On THIS machine the only
  way in and out is Valence. Other firmwares are never forced: Valence
  competes via the CLIENT ONRAMP (RFC-044). TCode passthrough is the easy rung
  (clients feed the TCode they already generate through a Valence session),
  then native segments, then native samples. First-party client support in
  MFP, Intiface and similar is maintained and encouraged. TCode integration is
  a CLIENT-SIDE adapter, never a hub-side stream.
- **Clients.** The MFP plugin and the verifier (`tools/valence_probe.py`)
  both live in the Valence repo. `LiveWireTest` refuses to run homed; run it
  TWICE back to back, which is the T3 check.

## The radio link: the C6 is a NIC, not a peer

`esp_hosted` over SDIO, host side on the P4. The C6 runs Espressif's stock
hosted slave image; we ship no C6 firmware. Sockets, lwIP, the WebSocket
server and every session live on the P4 (ruling 2026-09-20,
`governance.md` §6).

[verified 2026-09-20 -- console capture after flash: "Card init success" at
40 MHz 4-bit, "Identified slave [esp32c6]", slave firmware version 2.12.1,
`esp_wifi_init` ESP_OK at 2177 ms, STA up on three consecutive liveness lines,
0 SDIO timeouts, LP emitter `late=0` throughout]

- **Wiring, from M5's own example:** the Stamp-AddOn C6 snaps onto the
  Stamp-P4's 20-pin SDIO 3.0 header. CLK GPIO43, CMD GPIO44, D0 GPIO45,
  D1 GPIO46, D2 GPIO47, D3 GPIO48, RST GPIO42. All four data lines are wired,
  so 4-bit SDIO is available. The values live in
  `flagship_p4/sdkconfig.defaults` (C-1).
- **PIN TRAP, measured.** The public `CONFIG_ESP_HOSTED_SDIO_PIN_*` integers
  are promptless and DERIVED from `CONFIG_ESP_HOSTED_PRIV_SDIO_PIN_*_SLOT_1`.
  Setting only the public ones is SILENTLY IGNORED: the bus came up on the
  Espressif dev board's pins and timed out forty times. The `PRIV ..._SLOT_1`
  keys are the ones that carry the value. Generalize it: a Kconfig symbol with
  no prompt may be a derived view of another, and a config that "did not take"
  is a second symbol, not a wiring fault.
- **esp_hosted 2.x does NOT self-start under `esp_wifi_init`.** The order is
  `esp_hosted_init()`, `esp_hosted_connect_to_slave()`, then
  `esp_hosted_get_coprocessor_fwversion()` as the proof line, then
  `esp_wifi_init()`. The version line is the C-8 evidence that the slave is
  the thing being talked to.
- **The network costs 154 KB of internal SRAM** at default queue depths, none
  of it in PSRAM: 386,632 free before, 232,108 after hosted plus
  `esp_wifi_remote` plus netif plus lwIP connected. The MAC, PHY and packet
  buffers live on the C6. That number is the budget any queue-depth change is
  argued against (`memory-budget.md`).

## Legacy: the C5 bridge

The retired machine put the radio on a second ESP32 that TERMINATED the
WebSocket session and forwarded frames over a 4 Mbaud UART. Everything that
came with it -- COBS framing and the delimiter-resync property, the ESTOP
raw-scan duty, RX ring sizing against the drain interval, the physical-layer
common-ground and inversion footguns, and traps T31 (a gate must never disable
the transport carrying what it gates), T32 (fix a two-ended link at both ends;
pace retransmits on progress) and T33 (ARQ halves must match; a flash write
mutes a flash-resident RX ISR) -- is the case file in the archived SlopDrive-32
`.claude/rules/transport.md`. **None of it is reachable here:** the C6 is a
NIC, there is no second protocol terminator, and the P4 owns the socket. Read
that file before building any board-to-board link, and never re-derive it.

## T3 -- session teardown must be ONE funnel

**Rule:** every way a session can end (graceful bye, rude disconnect,
eviction, slot reuse) runs the same teardown routine, which runs the full
resource-loss policy. No unmonitored path to motion.
**Mechanism:** ownership released only by a watchdog that requires an occupied
slot means any teardown that clears the slot first leaks the ownership
forever, and every later client is silently rejected until reboot. Invisible
whenever deploys reboot the device between test runs.
**Mandatory check for any session-lifecycle change:** two full client sessions
back to back WITHOUT a reboot between them.

## T5 -- library callbacks run on the library's task

**Rule:** transport and network callbacks never mutate hub or session state
directly. They enqueue, and the owning task applies: attach, detach and RX are
deferred to the hub task.
**Mechanism:** the callback executes on the driver's or event loop's own
FreeRTOS task, concurrently with your update loop on another task. A callback
that nulls a pointer mid-`update()` is a use-after-free with no data race
visible in single-task reasoning. The hub is single-task BY DESIGN; keep it
true.

## T8 -- never stream to a wedged WebSocket client under a shared lock

**Rule:** do not send telemetry to backgrounded or unresponsive WS clients
from a path holding a mutex the server task needs. Sends must be bounded or
deferred.
**Mechanism:** a TCP send to a client that stopped reading fills the socket
buffer and blocks. If the sender holds the server mutex, every HTTP request
queues behind one dead browser tab.

## T11 -- wire-visible strings are protocol bytes

**Rule:** catalog descriptions, channel labels, and any string that ships in
an encoded artifact are wire content. Editing one changes encodings and etags
and invalidates binary fixtures. Respelling or rewording them is a protocol
change: C-11 flags it, frozen artifacts never change, and the Valence rename
rides an RFC for exactly this reason.

## T13 -- fan-out senders must re-check the transport, not just the session

**Rule:** any hub path that iterates slots and SENDS must test
`slot.transport != nullptr` on the slot it writes to. Occupied, ready and
subscribed do NOT imply attached. Skip the slot, and never route the miss
through the congestion or stall tracker.
**Mechanism:** a session whose transport dies is PARKED: slot, session_id,
grants and subscriptions are all RETAINED while `detachTransport()` nulls
`slot.transport`. The per-slot pumps skip null slots and are safe by
construction, but a FAN-OUT sender reaches the parked slot anyway and hands
null to an `ITransport&` parameter. Binding a null reference costs nothing
until the virtual call, which loads a vtable from address 0. It fires only
when one client sits parked while ANOTHER triggers the broadcast, so
single-client tests and clean-disconnect tests never see it, and the first
client of the next test run is the one that dies. Tracking the failed send is
its own bug: a parked session has no link to be congested on.
Bit the machine repo: the ONE fan-out sender missing the check its siblings
already had, three field reboots.

## T16 -- a stall watchdog sized for "stuck" cannot tell it apart from "big"

**Rule:** before arming a one-size stall or timeout watchdog on a shared queue,
classify the traffic crossing it. A healthy bulk transfer that legitimately
takes many ticks to drain looks IDENTICAL, from the queue's own point of view,
to a client that stopped reading and will never come back.
**Mechanism:** a control-stall timer exists to catch a client stranded waiting
on a reply that will never come. Before bulk chunk traffic got its own
backpressure class it was classified as ordinary control traffic: a 129-chunk
catalog transfer filled a 32-deep send queue well inside the timeout, and the
SAME timer built for a wedged client tore the session down mid-transfer, on a
perfectly healthy connection whose only sin was draining a big honest payload
no faster than the network allowed.
**Fix:** bulk chunks get their own class, paced against the registry's OWN
advertised sender budget via the same queue-depth check the data class already
used, and made to hold and retry. It never arms the control stall timer and
never NACKs or tears down on its own.
