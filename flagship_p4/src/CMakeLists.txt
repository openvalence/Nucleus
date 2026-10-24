# lib/valence is a SYMLINK to the sibling Valence checkout, pinned by
# valence.pin at the Nucleus root. The library ships no .c or .cpp, so an
# include path is the whole of consuming it: no component, no build step.
# READ-ONLY: a break there is fixed in flags here, never upstream.
#
# lib/strokeengine_patterns is VENDORED and header-only; its <Arduino.h> is
# the first-party adapter in patterns/arduino_compat (lib/strokeengine_patterns/
# VENDORED.md).
#
# Kinetic is the sibling checkout ../Kinetic beside this repo, pinned by
# kinetic.pin at the Nucleus root and READ-ONLY from here: a change lands in
# Kinetic first, then the pin moves. Nucleus plans with Kinetic²
# (include/kinetic2), header-only: an include path is the whole of consuming it.
# THE BOARD'S I/O HOSTS. Every door (ValenceMotorSwitch.h, ValenceEstopInput.h,
# ValenceFan.h, ValencePdSource.h, ValenceDriveLink.h, ValenceHomeSense.h) has
# one host per board, picked here: the Flagship's own, or on the barnacle
# (CONFIG_NUCLEUS_BOARD_BARNACLE, BoardPins.h) the helper link's
# (system/BarnacleLink.cpp) and the absent parts' (system/BarnacleBoard.cpp).
# lib/barnacle_link is the link's frames, header-only, shared with the shim.
if(CONFIG_NUCLEUS_BOARD_BARNACLE)
    set(board_io_srcs "system/BarnacleLink.cpp" "system/BarnacleBoard.cpp")
else()
    set(board_io_srcs "system/ValenceMotorSwitch.cpp" "system/ValenceDriveLink.cpp"
                      "system/ValenceEstopInput.cpp" "system/ValenceFan.cpp"
                      "system/ValencePdSource.cpp" "system/ValenceHomeSense.cpp")
endif()
idf_component_register(SRCS "main.cpp"
                            "hub/ValenceHub.cpp" "hub/ValenceDevice.cpp"
                            "hub/ValenceWsPort.cpp" "hub/ValenceEstopDatagram.cpp"
                            "hub/ValenceUiToken.cpp" "hub/ValenceDiscovery.cpp"
                            "hub/ValenceSerialPort.cpp" "hub/ValenceProvisioning.cpp"
                            "motion/ValenceMotion.cpp" "motion/MotionArbiter.cpp"
                            "patterns/ValencePattern.cpp" "patterns/PatternEngine.cpp"
                            "patterns/advanced/AdvancedPattern.cpp"
                            "system/ValenceHttp.cpp" "system/ValenceOta.cpp"
                            "system/ValenceDiag.cpp" "system/ValencePower.cpp"
                            "system/ValenceSelfCheck.cpp" "system/ValenceLogBridge.cpp"
                            "system/ValenceBoardIo.cpp"
                            "system/ValenceGlow.cpp"
                            "system/ValenceButtons.cpp"
                            "system/ValenceAccessoryIo.cpp"
                            "system/ValenceTcpTally.cpp"
                            ${board_io_srcs}
                       INCLUDE_DIRS "." "hub" "motion" "system"
                                    "../../lib/valence/include"
                                    "../../lib/geiger/include"
                                    "../../lib/flux/include"
                                    "../../lib/barnacle_link/include"
                                    "../../../Kinetic/include"
                                    "../../lib/strokeengine_patterns/include"
                                    "patterns/arduino_compat"
                       # REQUIRES: reached from a header in the PUBLIC include
                       # dirs above. esp_timer (ValencePlatform.h, geiger),
                       # esp_http_server (ValenceUiToken.h, ValenceWsPort.h),
                       # ulp (ulp_embed_binary and the ulp_main.h it generates).
                       REQUIRES esp_timer esp_http_server ulp
                       # PRIV_REQUIRES: reached from .cpp only. Do NOT rely on
                       # PlatformIO's SCons layer, which compiles this component
                       # with an all-components include set and so hides a
                       # missing entry from every pio build; ninja's own
                       # __idf_src target is the check.
                       PRIV_REQUIRES app_update esp_adc esp_driver_gpio esp_driver_i2c
                                     esp_driver_ledc esp_driver_pcnt esp_driver_rmt esp_driver_uart esp_mm
                                     esp_event esp_netif
                                     esp_wifi lwip nvs_flash spi_flash
                                     esp_driver_usb_serial_jtag
                                     valence_ble espressif__esp_hosted
                                     espressif__esp_wifi_remote)

# Float multiply-adds stay unfused (val-63b): GCC's default -ffp-contract=fast
# emits fmadd.s, and the planner must match kinetic.wasm and the native suite,
# both built -ffp-contract=off, bit for bit. Do not drop it for speed.
target_compile_options(${COMPONENT_LIB} PRIVATE -ffp-contract=off)

# lwIP's segment output hook (system/ValenceTcpHook.h): IDF includes the file
# named by ESP_IDF_LWIP_HOOK_FILENAME from lwip_default_hooks.h, so only the
# lwip component needs the define and the include path.
idf_component_get_property(lwip_lib lwip COMPONENT_LIB)
target_compile_definitions(${lwip_lib} PRIVATE "ESP_IDF_LWIP_HOOK_FILENAME=\"ValenceTcpHook.h\"")
target_include_directories(${lwip_lib} PRIVATE "${CMAKE_CURRENT_LIST_DIR}/system")

# ulp_embed_binary builds ulp/ with the LP core toolchain, embeds ulp_main.bin
# into the app, and generates ulp_main.h (every LP global as ulp_<name>). The
# app name MUST be ulp_main: pioarduino's ulp.py hardcodes it.
set(ulp_app_name ulp_main)
set(ulp_sources "../ulp/lp_quad.c")
set(ulp_exp_dep_srcs "main.cpp" "motion/ValenceMotion.cpp")
ulp_embed_binary(${ulp_app_name} "${ulp_sources}" "${ulp_exp_dep_srcs}")
