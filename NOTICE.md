# Notice

Nucleus (the firmware and the host device simulator in this repository) is
licensed under the CERN Open Hardware License version 2, Strongly Reciprocal
(CERN-OHL-S-2.0). The full text is in `LICENSE`.

Source location: the Nucleus repository in the openvalence organization on
GitHub, https://github.com/openvalence/Nucleus.

## What CERN-OHL-S-2.0 requires of a derivative

Section numbers are the license's own.

- Keep every notice: copyright, acknowledgment, source location, and the
  references to the license and its disclaimer (3.1, 3.2).
- A modified work must carry a notice stating that you modified it, with the
  date and a brief description of how (3.3b).
- The modified source, as a whole, is licensed under CERN-OHL-S-2.0 (3.3d).
  Including covered source in a larger work counts as modifying it, so the
  larger work becomes covered source (3.2).
- Available Components keep their own licenses and are excluded from that
  relicensing (3.3d). The third-party table below is that exclusion.
- Convey a product (a flashed board, a firmware image) only with the Complete
  Source or with notice of where to get it (4); a notice may require the
  source location to be displayed on the product or its documentation.
- The work is provided as is, with no warranty and no liability (6).

## Third-party components

| Component | Origin | License | Used by Nucleus for |
|---|---|---|---|
| OSSM-Lite (fray-d) | https://github.com/fray-d/OSSM-Lite | CERN-OHL-S-2.0 | The Advanced pattern modulators in `flagship_p4/src/hub/ValenceCatalog.h` (RFC-066), and the Advanced generator's stroke and modulation math in `flagship_p4/src/patterns/AdvancedPattern.*` and `PatternEngine.cpp` |
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
| Valence protocol (`spec/`) | https://github.com/openvalence/Valence | no license terms, see the Valence repository | The wire protocol Nucleus speaks; consumed by pinned sha in `valence.pin` |
| Valence library and JS client (`lib/valence`) | https://github.com/openvalence/Valence | MIT, per `Valence/LICENSE` | `lib/valence`, the Valence library, pinned by sha in `valence.pin` |
| Flux, Geiger, Kinetic | this repository | first-party, CERN-OHL-S-2.0 | `lib/flux` (LED grammar), `lib/geiger` (logging), `lib/kinetic` (motion planner) |

The managed components and ESP-IDF are Available Components in the sense of
the license: they keep their own terms. `flagship_p4/managed_components/` is
git-ignored; `flagship_p4/dependencies.lock` pins the versions above.

No other copied or adapted code with an attribution marker exists in
`flagship_p4/src`, `sim`, `flagship_ch32v003`, `test` or `tools`.
