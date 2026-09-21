# Nucleus

Firmware for the **OSSM Flagship**: an extensible, high-performance modular
linear motion control platform.

Three names, three things:

- **OSSM Flagship** -- the PCB.
- **Nucleus** (full name Valence Nucleus) -- this firmware.
- **Valence** -- the protocol it speaks. It is developed in the sibling
  `Valence` repo and consumed here READ-ONLY, pinned by sha in `valence.pin`.
  Wire-visible names landed upstream with RFC-060; this repo follows the pin,
  and never edits the spec to match its code.

## The boards

The **ESP32-P4** owns everything that decides: motion, the Valence hub,
policy, and the sockets. Its LP core renders quadrature edges and its signed
edge count is the machine's position; the HP core evaluates the plan and runs
the hub. The **ESP32-C6** is a WiFi NIC over SDIO, running Espressif's stock
`esp_hosted` slave image. There is no bridge and no second firmware of ours.

## Build and flash

Pure ESP-IDF, one PlatformIO project per chip. From PowerShell, never Git Bash:

```
pio run -d flagship_p4              # build
pio run -d flagship_p4 -t upload    # flash (COM15 on this bench)
```

Config lives in `flagship_p4/sdkconfig.defaults` (hand-written) and
`flagship_p4/platformio.ini`. The generated `sdkconfig.<env>` is derived; do
not edit it. **Any change that moves the memory map needs a clean build:
delete `.pio`.** That trap and the rest are in
`.claude/rules/build-test-deploy.md`.

## Where the rules live

- `.claude/rules/governance.md` -- the law (C-1..C-12), the Map of Truth, the
  Canon Flag protocol, and the amendments log. Read it first.
- `.claude/rules/*.md` -- engineering doctrine: architecture, memory safety,
  style, motion control, transport, memory budget, logging and LEDs, build and
  deploy, layout, navigation.
- `tools/canon_lint.py` -- the mechanical floor. Run it before calling
  substantive work done; zero findings is the bar.
- The dev board (`bd`, `val-` prefix) is the sole home for volatile truth:
  versions, deployment state, milestones, open bugs, pending rulings. Status
  never goes into prose.

## The machine repo

`../SlopDrive-32` is the archived three-board machine (ESP32-S3 + ESP32-C5 +
RP2350) and stays where it is. It is the reference this board is measured
against and the case file for every trap this firmware does not inherit: the
UART bridge, the AsyncTCP lifetime hunt, the heap-corruption record. Its rules
files are cited by path from ours; they are never copied.
