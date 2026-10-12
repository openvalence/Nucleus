// esp_timer.h -- host stand-in for test_ws_port: the suite's fake clock
#pragma once

#include <cstdint>

int64_t esp_timer_get_time();
