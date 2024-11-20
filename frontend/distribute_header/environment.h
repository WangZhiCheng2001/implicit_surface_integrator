#pragma once

#include <stdint.h>

typedef struct sSetting {
    uint32_t resolution;
} setting_descriptor;

extern "C" __declspec(dllimport) void update_setting(const setting_descriptor desc);
// apply updated settings to the environment
extern "C" __declspec(dllimport) void update_environment();