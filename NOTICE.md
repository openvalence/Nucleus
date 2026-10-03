# Notice

Nucleus
Copyright 2026 the Nucleus authors

This file is the NOTICE file of Apache License 2.0 section 4(d): a
redistribution of Nucleus or of a work derived from it carries the
attribution notices in this file.

Nucleus (the firmware and the host device simulator in this repository) is
licensed under the Apache License, Version 2.0, full text in `LICENSE`, with
one exception: `flagship_p4/src/patterns/advanced/` is derived from fray-d's
OSSM-Lite and stays under the CERN Open Hardware License version 2, Strongly
Reciprocal (CERN-OHL-S-2.0), with its own `LICENSE` and `NOTICE.md` there.

Source location: https://github.com/openvalence/Nucleus.

## How the two licenses meet

Section numbers are CERN-OHL-S-2.0's own.

- The carve-out is complete source under CERN-OHL-S-2.0, so it is an
  Available Component (1.7(a)). A firmware image that includes it is a
  Product of that unit; whoever conveys the image ships the Complete Source
  or notice of its Source Location (4), and this repository is that location.
- The Apache-2.0 code is an Available Component of such a Product as well
  (1.7(a) through 1.2(c)), so it keeps its own license and is not relicensed
  (3.3(d)).
- A build without `flagship_p4/src/patterns/advanced/` carries no
  CERN-OHL-S-2.0 obligation.

## Third-party components

| Component | Origin | License | Used by Nucleus for |
|---|---|---|---|
| OSSM-Lite (fray-d) | https://github.com/fray-d/OSSM-Lite | CERN-OHL-S-2.0 | The Advanced generator's stroke and modulation math, carved out in `flagship_p4/src/patterns/advanced/` (its `NOTICE.md` lists the modifications). The modulator field definitions in `flagship_p4/src/hub/ValenceCatalog.h` (RFC-066) are Valence interface vocabulary, credited there, and stay Apache-2.0 |
| Ruckig Community Version 0.19.4 | https://github.com/pantor/ruckig | MIT, Copyright (c) 2021 Lars Berscheid | `lib/ruckig`, vendored unmodified (`lib/ruckig/VENDORED.md`); the jerk-limited trajectory solver under `lib/kinetic` |
| StrokeEngine patterns (theelims) | https://github.com/theelims/StrokeEngine, as carried by https://github.com/KinkyMakers/OSSM-hardware (`Software/lib/StrokeEngine/src/`) | MIT, Copyright (C) 2021 theelims | `lib/strokeengine_patterns`, the seven core stroke patterns (`lib/strokeengine_patterns/VENDORED.md`) |
| ESP-IDF 5.5.4 | https://github.com/espressif/esp-idf | Apache-2.0 | The framework the P4 firmware builds on (`flagship_p4`); fetched at build time, not stored in this repository |
| espressif/esp_hosted 2.12.13 | https://components.espressif.com/components/espressif/esp_hosted | Apache-2.0 | Host side of the C6 WiFi link; the C6 runs Espressif's stock slave image (managed component) |
| espressif/esp_wifi_remote 1.6.4 | https://components.espressif.com/components/espressif/esp_wifi_remote | Apache-2.0 | WiFi API forwarded to the C6 (managed component) |
| espressif/wifi_remote_over_eppp 0.3.3 | https://components.espressif.com/components/espressif/wifi_remote_over_eppp | Apache-2.0 (per its `idf_component.yml`; the package ships no LICENSE file) | Transport glue for esp_wifi_remote (managed component) |
| espressif/eppp_link 1.1.6 | https://components.espressif.com/components/espressif/eppp_link | Apache-2.0 | PPP link between the P4 and the C6 (managed component) |
| espressif/esp_serial_slave_link 1.1.2 | https://components.espressif.com/components/espressif/esp_serial_slave_link | Apache-2.0 | SDIO link driver under eppp_link (managed component) |
| IXWebSocket 11.4.6 | https://github.com/machinezone/IXWebSocket | BSD-3-Clause, Copyright (c) 2018 Machine Zone, Inc. | WebSocket server of the host simulator `sim/valencesim` only; fetched by CMake, not in the firmware |
| doctest 2.4.12 | https://github.com/doctest/doctest | MIT (per its `library.json`; no LICENSE file in the fetched package) | Native test runner only (`pio test -e native`); not in any shipped image |
| ch32fun (PlatformIO `ch32v003fun` framework) | https://github.com/cnlohr/ch32fun | unknown (not fetched on this host) | The `flagship_ch32v003` monitor firmware build |
| Valence protocol (`spec/`) | https://github.com/openvalence/Valence | CC BY 4.0 (Valence/LICENSE-SPEC) | The wire protocol Nucleus speaks; consumed by pinned sha in `valence.pin` |
| Valence library and JS client (`lib/valence`) | https://github.com/openvalence/Valence | MIT, per `Valence/LICENSE` | `lib/valence`, the Valence library, pinned by sha in `valence.pin` |
| Flux, Geiger, Kinetic | this repository | first-party, Apache-2.0 | `lib/flux` (LED grammar), `lib/geiger` (logging), `lib/kinetic` (motion planner) |

`flagship_p4/managed_components/` is git-ignored;
`flagship_p4/dependencies.lock` pins the versions above.

No other copied or adapted code with an attribution marker exists in
`flagship_p4/src`, `sim`, `flagship_ch32v003`, `test` or `tools`.
