# lib/valence is a SYMLINK to the sibling Valence checkout, pinned by
# valence.pin at the Nucleus root. The library ships no .c or .cpp, so an
# include path is the whole of consuming it: no component, no build step.
# READ-ONLY: a break there is fixed in flags here, never upstream.
#
# lib/kinetic is header-only too, but the vendored Ruckig it wraps is not: its
# step solvers are real translation units and are listed here. lib/ruckig is
# BYTE-IDENTICAL to upstream -- wrap, never patch (lib/ruckig/VENDORED.md).
idf_component_register(SRCS "main.cpp"
                            "hub/ValenceHub.cpp" "hub/ValenceWsPort.cpp"
                            "hub/ValenceUiToken.cpp"
                            "motion/ValenceMotion.cpp"
                            "system/ValenceHttp.cpp" "system/ValenceOta.cpp"
                            "../../lib/ruckig/src/ruckig/brake.cpp"
                            "../../lib/ruckig/src/ruckig/position_first_step1.cpp"
                            "../../lib/ruckig/src/ruckig/position_first_step2.cpp"
                            "../../lib/ruckig/src/ruckig/position_second_step1.cpp"
                            "../../lib/ruckig/src/ruckig/position_second_step2.cpp"
                            "../../lib/ruckig/src/ruckig/position_third_step1.cpp"
                            "../../lib/ruckig/src/ruckig/position_third_step2.cpp"
                            "../../lib/ruckig/src/ruckig/velocity_second_step1.cpp"
                            "../../lib/ruckig/src/ruckig/velocity_second_step2.cpp"
                            "../../lib/ruckig/src/ruckig/velocity_third_step1.cpp"
                            "../../lib/ruckig/src/ruckig/velocity_third_step2.cpp"
                       INCLUDE_DIRS "." "hub" "motion" "system"
                                    "../../lib/valence/include"
                                    "../../lib/geiger/include"
                                    "../../lib/kinetic/include"
                                    "../../lib/ruckig/include"
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
                       PRIV_REQUIRES app_update esp_driver_parlio esp_event esp_netif
                                     esp_wifi lwip nvs_flash spi_flash
                                     espressif__esp_hosted
                                     espressif__esp_wifi_remote)

# ulp_embed_binary builds ulp/ with the LP core toolchain, embeds ulp_main.bin
# into the app, and generates ulp_main.h (every LP global as ulp_<name>). The
# app name MUST be ulp_main: pioarduino's ulp.py hardcodes it.
set(ulp_app_name ulp_main)
set(ulp_sources "../ulp/lp_quad.c")
set(ulp_exp_dep_srcs "main.cpp" "motion/ValenceMotion.cpp")
ulp_embed_binary(${ulp_app_name} "${ulp_sources}" "${ulp_exp_dep_srcs}")
