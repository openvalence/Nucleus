// secrets.example.h -- copy to secrets.h (gitignored) and fill in
#pragma once
#define SECRET_WIFI_SSID     "your-ssid"
#define SECRET_WIFI_PASSWORD "your-password"
// The OTA bearer for POST /ota on :80. Compared constant-time against the
// X-OTA-Token header; a LAN host that has it can overwrite this board's
// firmware, so make it long and random (python -c "import secrets;
// print(secrets.token_hex(24))"). tools/ota.py reads it out of secrets.h.
#define SECRET_OTA_TOKEN     "replace-me-with-48-random-hex-characters"
