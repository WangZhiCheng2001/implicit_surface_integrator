#pragma once

#include <stdbool.h>

#include <macros.h>
#include <blobtree.h>

typedef struct {
    const raw_vector3d_t* vertices;
    const uint32_t*       faces;         // indices of vertices in each face
    const uint32_t*       vertex_counts; // number of vertices in each face
    uint32_t              num_vertices;
    uint32_t              num_faces;
} polymesh_t;

typedef struct {
    polymesh_t mesh;
    double     surf_int_result;
    double     vol_int_result;
    bool       success;
} solve_result_t;

EXTERN_C API solve_result_t execute_solver(const virtual_node_t* tree_node);
// output time usage statistics to console
EXTERN_C API void           print_statistics();
EXTERN_C API void           clear_statistics();