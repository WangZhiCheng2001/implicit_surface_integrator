#pragma once

#include "primitive_descriptor.h"

typedef struct {
    primitive_type type;
    void*          desc; // Type conversion when using
} primitive_node_t;

// almost same as node_t, but has parent's and children's pointers to indicate
// the hierarchy, and it is outside of the tree
typedef struct {
    uint32_t main_index;
    uint32_t inner_index;
} virtual_node_t;
