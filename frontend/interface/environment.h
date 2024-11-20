#pragma once

#include <stdint.h>

#include <macros.h>

typedef struct sSetting {
    uint32_t resolution;
} setting_descriptor;

EXTERN_C API void update_setting(const setting_descriptor desc);
// apply updated settings to the environment
EXTERN_C API void update_environment();