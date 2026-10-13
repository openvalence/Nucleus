# The BLE GATT binding's NimBLE half (ValenceBle.cpp; the hardware-free pipe is
# src/hub/ValenceBle.h). A COMPONENT OF ITS OWN, AND THAT IS LOAD-BEARING:
# pioarduino's ulp.py hands the ULP build every include path of the component
# that embeds ulp_main (src), on one command line. The bt component's NimBLE
# include set pushed that line past Windows' limit ("The filename or extension
# is too long", measured 2026-10-13). Here, bt stays a private requirement
# of this component and never reaches src's include set.
idf_component_register(SRCS "ValenceBle.cpp"
                       PRIV_INCLUDE_DIRS "../../src" "../../src/hub"
                                         "../../../lib/valence/include"
                                         "../../../lib/geiger/include"
                       PRIV_REQUIRES bt esp_timer espressif__esp_hosted)
