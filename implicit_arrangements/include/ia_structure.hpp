#pragma once

#include <array>

#include <implicit_arrangement.h>
#include "common_structure.hpp"
#include "container/dynamic_bitset.hpp"

/// forward declaration
struct Point2D;
struct Point3D;

namespace detail
{
template <size_t N>
struct deduce_vertex_type;

template <>
struct deduce_vertex_type<2> {
    using type = Point2D;
};

template <>
struct deduce_vertex_type<3> {
    using type = Point3D;
};
} // namespace detail

template <size_t N>
using IAVertex = typename detail::deduce_vertex_type<N>::type;

template <size_t N>
struct IAEdge;

template <>
struct IAEdge<2> {
    std::array<uint32_t, 2> vertices{INVALID_INDEX, INVALID_INDEX}; ///< ordered
    uint32_t                supporting_plane{INVALID_INDEX};
    uint32_t                positive_face{INVALID_INDEX};
    uint32_t                negative_face{INVALID_INDEX};
};

template <>
struct IAEdge<3> {
    std::array<uint32_t, 2> vertices{INVALID_INDEX, INVALID_INDEX}; ///< ordered
    std::array<uint32_t, 2> supporting_planes{INVALID_INDEX, INVALID_INDEX};
};

template <size_t N>
struct IAFace;

template <>
struct IAFace<2> {
    small_vector_mp<uint32_t> edges{}; ///< ordered
    small_dynamic_bitset_mp<> signs{}; ///< sign of implicit functions
};

template <>
struct IAFace<3> {
    small_vector_mp<uint32_t> edges{}; ///< ordered
    uint32_t                  supporting_plane{INVALID_INDEX};
    uint32_t                  positive_cell{INVALID_INDEX};
    uint32_t                  negative_cell{INVALID_INDEX};
};

template <size_t N>
struct IACell;

template <>
struct IACell<3> {
    small_vector_mp<uint32_t> faces{}; ///< ordered
    small_dynamic_bitset_mp<> signs{}; ///< sign of implicit functions
};

template <size_t N>
struct IAComplex;

template <>
struct IAComplex<2> {
    small_vector_mp<IAVertex<2>> vertices{};
    small_vector_mp<IAEdge<2>>   edges{};
    small_vector_mp<IAFace<2>>   faces{};
};

template <>
struct IAComplex<3> {
    small_vector_mp<IAVertex<3>> vertices{};
    small_vector_mp<IAEdge<3>>   edges{};
    small_vector_mp<IAFace<3>>   faces{};
    small_vector_mp<IACell<3>>   cells{};
};