#pragma once

#include <stdbool.h>

#include <blobtree.h>

typedef struct {
    raw_vector3d_t* vertices;
    uint32_t*       faces;         // indices of vertices in each face
    uint32_t*       vertex_counts; // number of vertices in each face
    uint32_t        num_vertices;
    uint32_t        num_faces;
} polymesh_t;

typedef struct {
    polymesh_t mesh;
    double     surf_int_result;
    double     vol_int_result;
    bool       success;
} solve_result_t;

extern "C" __declspec(dllimport) solve_result_t execute_solver(const virtual_node_t* tree_node);
// output time usage statistics to console
extern "C" __declspec(dllimport) void           print_statistics();
extern "C" __declspec(dllimport) void           clear_statistics();