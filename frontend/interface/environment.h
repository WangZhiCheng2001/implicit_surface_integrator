#pragma once

#include <stdint.h>

#include <macros.h>
#include <blobtree.h>

typedef struct sSetting {
    uint32_t resolution;        // will split the background mesh into (resolution + 1)^3 grids
    double   scene_aabb_margin; // margin to add to the scene AABB to avoid artifacts
} setting_descriptor;

EXTERN_C API void update_setting(const setting_descriptor desc);
// apply updated settings to the environment
EXTERN_C API void update_environment(const virtual_node_t* tree_node);