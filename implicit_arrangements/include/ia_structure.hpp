#pragma once

#include <array>

#include <implicit_arrangement.hpp>

using ia_vertex_t = point_t;

struct ia_edge_t {
    std::array<uint32_t, 2> vertices{INVALID_INDEX, INVALID_INDEX}; ///< ordered
    std::array<uint32_t, 2> supporting_planes{INVALID_INDEX, INVALID_INDEX};
};

struct ia_face_t {
    stl_vector_mp<uint32_t> edges{}; ///< ordered
    uint32_t                supporting_plane{INVALID_INDEX};
    uint32_t                positive_cell{INVALID_INDEX};
    uint32_t                negative_cell{INVALID_INDEX};
};

struct ia_cell_t {
    stl_vector_mp<uint32_t> faces{}; ///< ordered
    stl_vector_mp<bool>     signs{}; ///< sign of implicit functions
};

struct ia_complex_t {
    stl_vector_mp<ia_vertex_t> vertices{};
    stl_vector_mp<ia_edge_t>   edges{};
    stl_vector_mp<ia_face_t>   faces{};
    stl_vector_mp<ia_cell_t>   cells{};
};