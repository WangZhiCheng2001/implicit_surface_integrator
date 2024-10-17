#pragma once

#include <stdint.h>
#include <array>

#include <tbb/tbb_allocator.h>

#include <container/small_vector.hpp>
#include <glue/eigen_alias.hpp>

using raw_point_t = Eigen::Vector3d;

typedef struct TriangleVertexIndices {
    uint32_t v1, v2, v3;
} triangle_vertex_indices_t;

typedef struct TetrahedronVertexIndices {
    uint32_t v1, v2, v3, v4;
} tetrahedron_vertex_indices_t;

typedef struct TetrahedronMeshDescriptor {
    std::vector<raw_point_t, tbb::tbb_allocator<raw_point_t>>                                   vertices{};
    std::vector<tetrahedron_vertex_indices_t, tbb::tbb_allocator<tetrahedron_vertex_indices_t>> indices{};
} tetrahedron_mesh_t;

typedef struct VertexDescriptor {
    uint32_t volume_index;        /// volume (usually tetrahedron) index in the global list of volumes
    uint16_t local_vertex_index;  /// vertex index in the volume
    uint8_t minimal_simplex_flag; /// minimal simplex that contains the IsoVert (1: point, 2: edge, 3: triangle, 4: tetrahedron)
} vertex_header_t;

typedef struct FaceDescriptor {
    uint32_t volume_index;     /// volume (usually tetrahedron) index in the global list of volumes
    uint32_t local_face_index; /// face index in the volume
} face_header_t;

typedef struct EdgeDescriptor {
    uint32_t face_index;       /// face index in the global list of faces
    uint32_t local_edge_index; /// edge index in the face
} edge_header_t;

static constexpr uint16_t no_implicit_function = std::numeric_limits<uint16_t>::max();

/// a polygon in a polygonal mesh
struct PolygonFace {
    std::vector<uint32_t>      vertex_indices; /// a list of polygon's vertex indices (index into some global list of vertices)
    std::vector<face_header_t> headers;        /// the local index of this polygon in all the tets that contains it
    size_t implicit_function_index; /// function index of the implicit function which its isosurface contains the polygon.
};

/// vertex of isosurface
struct IsoVertex {
    vertex_header_t         header;
    std::array<size_t, 4>   simplex_vertex_indices; /// ? index into a list of tet vertices
    std::array<uint16_t, 3> implicit_function_indices = {
        no_implicit_function,
        no_implicit_function,
        no_implicit_function}; /// list of implicit functions whose isosurfaces pass IsoVert (indexed into a global list of
    /// implicit functions)
    /// so current solution only supports at most 3 iso-surfaces passing through a vertex?
};

/// Edge of the isosurface
struct IsoEdge {
    /// Two end-vertics' indices
    size_t                     v1, v2;
    std::vector<edge_header_t> headers;
};