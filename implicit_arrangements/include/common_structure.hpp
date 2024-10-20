#pragma once

#include <container/small_vector.hpp>

/**
 * A 2D edge is defined by 3 implicit hyperplanes.  The first end point is
 * the intersection of `prev_plane` and `curr_plane`.  The second end point is
 * the intersection of `next_plane` and `curr_plane`.
 */
template <size_t N>
struct Edge;

template <>
struct Edge<2> {
    uint32_t prev_plane, curr_plane, next_plane;
};

template <>
struct Edge<3> {
    uint32_t supporting_plane;
    uint32_t prev_plane, curr_plane, next_plane;
};

template <size_t N>
struct Face;

template <>
struct Face<3> {
    small_vector_mp<uint32_t> edge_planes{};
    uint32_t                  supporting_plane;
};

template <size_t N>
struct Cell;

template <>
struct Cell<2> {
    small_vector_mp<uint32_t> edges{};
};

template <>
struct Cell<3> {
    small_vector_mp<uint32_t> faces{};
};

static constexpr auto INVALID_INDEX = std::numeric_limits<uint32_t>::max();