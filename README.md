# Nucleus

Nucleus is the firmware for the OSSM Flagship linear motion controller.

Names:

- **OSSM Flagship** -- the PCB.
- **Nucleus** (full name Valence Nucleus) -- this firmware.
- **Valence** -- the protocol it speaks. It is developed in the sibling
  `Valence` repo and consumed here READ-ONLY, pinned by sha in `valence.pin`.
  Wire-visible names are defined by RFC-060. This repo does not edit the
  spec; it updates to a new pin.

## Chips

The **ESP32-P4** runs motion, the Valence hub,
policy and the sockets. Its LP core renders quadrature edges and its signed
edge count is the machine's position; the HP core evaluates the plan and runs
the hub. The **ESP32-C6** is a WiFi NIC over SDIO, running Espressif's stock
`esp_hosted` slave image. The C6 runs no OpenValence code. The board monitor, a CH32V003 (U12), runs its
own firmware from `flagship_ch32v003/`; the P4 programs it over SWIO and
carries its image inside the Nucleus OTA.
On the bench, homing triggers from an external current-sense board.

## Build and flash

The P4 firmware is pure ESP-IDF. Each chip has its own PlatformIO project. Build from PowerShell, never Git Bash:

```
pio run -d flagship_p4              # build
pio run -d flagship_p4 -t upload    # flash (COM15 on this bench; set `upload_port` in `flagship_p4/platformio.ini`)
```

Config lives in `flagship_p4/sdkconfig.defaults` (hand-written) and
`flagship_p4/platformio.ini`. The generated `sdkconfig.<env>` is derived; do
not edit it. **Any change that moves the memory map needs a clean build:
delete `.pio`.** Other build constraints: `.claude/rules/build-test-deploy.md`.

## Doctrine

- `.claude/rules/governance.md` -- the Canon (C-1..C-12), the Map of Truth, the
  Canon Flag protocol, and the amendments log. Read it first.
- `.claude/rules/*.md` -- engineering doctrine: architecture, memory safety,
  style, motion control, transport, memory budget, logging and LEDs, build and
  deploy, layout, navigation.
- `tools/canon_lint.py` -- the lint that checks the Canon's mechanical rules. Run it before calling
  substantive work done; the expected result is zero findings.
- The dev board (`bd`, prefix `val-`) is the sole home for volatile truth:
  versions, deployment state, milestones, open bugs, pending rulings; the docs do not
  record status.

## License

Nucleus is licensed under Apache-2.0 (`LICENSE`), except
`flagship_p4/src/patterns/advanced/`, which is derived from fray-d's OSSM-Lite
and stays CERN-OHL-S-2.0 (its own `LICENSE` and `NOTICE.md`). Third-party
components keep their own licenses; `NOTICE.md` lists each with its origin and
what uses it, and is the Apache NOTICE file.
