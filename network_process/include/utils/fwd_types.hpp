#pragma once

#include <stdint.h>
#include <array>

#include <container/small_vector.hpp>
#include "eigen_alias.hpp"

using raw_point_t = Eigen::Vector3d;

struct triangle_vertex_indices_t {
    uint32_t v1, v2, v3;
};

struct tetrahedron_vertex_indices_t {
    uint32_t v1, v2, v3, v4;
};

struct tetrahedron_mesh_t {
    small_vector_mp<raw_point_t, 8 * 8 * 8>                      vertices{};
    small_vector_mp<tetrahedron_vertex_indices_t, 7 * 7 * 7 * 5> indices{};
};

struct vertex_header_t {
    uint32_t volume_index{};       /// volume (usually tetrahedron) index in the global list of volumes
    uint32_t local_vertex_index{}; /// vertex index in the volume
    uint8_t
        minimal_simplex_flag{}; /// minimal simplex that contains the IsoVert (1: point, 2: edge, 3: triangle, 4: tetrahedron)

    vertex_header_t() = default;

    vertex_header_t(uint32_t volume_index, uint32_t local_vertex_index, uint8_t minimal_simplex_flag)
        : volume_index(volume_index), local_vertex_index(local_vertex_index), minimal_simplex_flag(minimal_simplex_flag)
    {
    }

    vertex_header_t(const vertex_header_t&)            = default;
    vertex_header_t(vertex_header_t&&)                 = default;
    vertex_header_t& operator=(const vertex_header_t&) = default;
    vertex_header_t& operator=(vertex_header_t&&)      = default;
};

struct face_header_t {
    uint32_t volume_index{};     /// volume (usually tetrahedron) index in the global list of volumes
    uint32_t local_face_index{}; /// face index in the volume

    face_header_t() = default;

    face_header_t(uint32_t volume_index, uint32_t local_face_index)
        : volume_index(volume_index), local_face_index(local_face_index)
    {
    }

    face_header_t(const face_header_t&)            = default;
    face_header_t(face_header_t&&)                 = default;
    face_header_t& operator=(const face_header_t&) = default;
    face_header_t& operator=(face_header_t&&)      = default;
};

struct edge_header_t {
    uint32_t face_index{};       /// face index in the global list of faces
    uint32_t local_edge_index{}; /// edge index in the face

    edge_header_t() = default;

    edge_header_t(uint32_t face_index, uint32_t local_edge_index) : face_index(face_index), local_edge_index(local_edge_index)
    {
    }

    edge_header_t(const edge_header_t&)            = default;
    edge_header_t(edge_header_t&&)                 = default;
    edge_header_t& operator=(const edge_header_t&) = default;
    edge_header_t& operator=(edge_header_t&&)      = default;
};

template <size_t N>
using pod_key_t = std::array<uint32_t, N>;

static constexpr uint16_t no_implicit_function = std::numeric_limits<uint16_t>::max();
static constexpr uint32_t invalid_index        = std::numeric_limits<uint32_t>::max();

/// a polygon in a polygonal mesh
struct PolygonFace {
    small_vector_mp<uint32_t, 4>
        vertex_indices{}; /// a list of polygon's vertex indices (index into some global list of vertices)
    small_vector_mp<face_header_t, 2> headers{}; /// the local index of this polygon in all the tets that contains it
    uint32_t implicit_function_index{}; /// function index of the implicit function which its isosurface contains the polygon.
};

/// vertex of isosurface
struct IsoVertex {
    vertex_header_t         header{};
    std::array<uint32_t, 4> simplex_vertex_indices{}; /// ? index into a list of tet vertices
    std::array<uint32_t, 3> implicit_function_indices = {
        no_implicit_function,
        no_implicit_function,
        no_implicit_function}; /// list of implicit functions whose isosurfaces pass IsoVert (indexed into a global list of
    /// implicit functions)
    /// so current solution only supports at most 3 iso-surfaces passing through a vertex?
};

/// Edge of the isosurface
struct IsoEdge {
    /// Two end-vertics' indices
    uint32_t                       v1{}, v2{};
    small_vector_mp<edge_header_t> headers{};
};

template <typename T>
using stl_vector_mp = detail::stl_vector_bind<ScalableMemoryPoolAllocator>::type<T>;

namespace std
{
template <>
struct hash<vertex_header_t> {
    size_t operator()(const vertex_header_t& v) const
    {
        return std::hash<uint32_t>()(v.volume_index) ^ std::hash<uint32_t>()(v.local_vertex_index)
               ^ std::hash<uint8_t>()(v.minimal_simplex_flag);
    }
};

template <>
struct hash<face_header_t> {
    size_t operator()(const face_header_t& f) const
    {
        return std::hash<uint32_t>()(f.volume_index) ^ std::hash<uint32_t>()(f.local_face_index);
    }
};

template <>
struct hash<edge_header_t> {
    size_t operator()(const edge_header_t& e) const
    {
        return std::hash<uint32_t>()(e.face_index) ^ std::hash<uint32_t>()(e.local_edge_index);
    }
};

template <size_t N>
struct hash<pod_key_t<N>> {
    size_t operator()(const pod_key_t<N>& k) const
    {
        size_t res = 0;
        for (size_t i = 0; i < N; ++i) { res ^= std::hash<uint32_t>()(k[i]); }
        return res;
    }
};

template <>
struct equal_to<vertex_header_t> {
    bool operator()(const vertex_header_t& v1, const vertex_header_t& v2) const
    {
        return v1.volume_index == v2.volume_index && v1.local_vertex_index == v2.local_vertex_index
               && v1.minimal_simplex_flag == v2.minimal_simplex_flag;
    }
};

template <>
struct equal_to<face_header_t> {
    bool operator()(const face_header_t& f1, const face_header_t& f2) const
    {
        return f1.volume_index == f2.volume_index && f1.local_face_index == f2.local_face_index;
    }
};

template <>
struct equal_to<edge_header_t> {
    bool operator()(const edge_header_t& e1, const edge_header_t& e2) const
    {
        return e1.face_index == e2.face_index && e1.local_edge_index == e2.local_edge_index;
    }
};

template <size_t N>
struct equal_to<pod_key_t<N>> {
    bool operator()(const pod_key_t<N>& k1, const pod_key_t<N>& k2) const
    {
        for (size_t i = 0; i < N; ++i) {
            if (k1[i] != k2[i]) { return false; }
            return true;
        }

        return true;
    }
};
} // namespace std