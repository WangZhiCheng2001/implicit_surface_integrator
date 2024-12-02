#pragma once

#include <container/hashmap.hpp>

#include <utils/fwd_types.hpp>

struct simplified_vertex_header_t {
    uint32_t volume_index{};
    uint32_t local_vertex_index{};
};

struct component_header_t {
    uint32_t vertex_index{};
    uint32_t component_index{};
};

struct face_with_orient_t {
    uint32_t face_id{};
    int8_t   orient{};
};

namespace std
{
template <>
struct hash<simplified_vertex_header_t> {
    size_t operator()(const simplified_vertex_header_t &k) const
    {
        return (static_cast<size_t>(k.volume_index) << 32) | k.local_vertex_index;
        // return std::hash<uint32_t>()(k.volume_index) ^ std::hash<uint32_t>()(k.local_vertex_index);
    }
};

template <>
struct hash<component_header_t> {
    size_t operator()(const component_header_t &k) const
    {
        return (static_cast<size_t>(k.vertex_index) << 32) | k.component_index;
        // return std::hash<uint32_t>()(k.vertex_index) ^ std::hash<uint32_t>()(k.component_index);
    }
};

template <>
struct hash<face_with_orient_t> {
    size_t operator()(const face_with_orient_t &k) const
    {
        return (static_cast<size_t>(k.face_id) << 32) | k.orient;
        // return std::hash<uint32_t>()(k.face_id) ^ std::hash<int8_t>()(k.orient);
    }
};

template <>
struct equal_to<simplified_vertex_header_t> {
    bool operator()(const simplified_vertex_header_t &k1, const simplified_vertex_header_t &k2) const
    {
        return k1.volume_index == k2.volume_index && k1.local_vertex_index == k2.local_vertex_index;
    }
};

template <>
struct equal_to<component_header_t> {
    bool operator()(const component_header_t &k1, const component_header_t &k2) const
    {
        return k1.vertex_index == k2.vertex_index && k1.component_index == k2.component_index;
    }
};

template <>
struct equal_to<face_with_orient_t> {
    bool operator()(const face_with_orient_t &k1, const face_with_orient_t &k2) const
    {
        return k1.face_id == k2.face_id && k1.orient == k2.orient;
    }
};
} // namespace std

// topological ray shooting for implicit arrangement
ISNP_API void topo_ray_shooting(const tetrahedron_mesh_t                            &tet_mesh,
                                const stl_vector_mp<std::shared_ptr<arrangement_t>> &cut_results,
                                const stl_vector_mp<iso_vertex_t>                   &iso_verts,
                                const stl_vector_mp<polygon_face_t>                 &iso_faces,
                                const stl_vector_mp<stl_vector_mp<uint32_t>>        &patches,
                                const stl_vector_mp<uint32_t>                       &patch_of_face,
                                const stl_vector_mp<stl_vector_mp<uint32_t>>        &shells,
                                const stl_vector_mp<uint32_t>                       &shell_of_half_patch,
                                const stl_vector_mp<stl_vector_mp<uint32_t>>        &components,
                                const stl_vector_mp<uint32_t>                       &component_of_patch,
                                stl_vector_mp<std::pair<uint32_t, uint32_t>>        &shell_links);

// Given tet mesh,
// build the map: v-->v_next, where v_next has lower order than v
ISNP_API void build_next_vert(const tetrahedron_mesh_t &tet_mesh, stl_vector_mp<uint32_t> &next_vert);

// find extremal edge for each component
ISNP_API void find_extremal_edges(const stl_vector_mp<raw_point_t>                                 &pts,
                                  const stl_vector_mp<iso_vertex_t>                                &iso_verts,
                                  const stl_vector_mp<polygon_face_t>                              &iso_faces,
                                  const stl_vector_mp<stl_vector_mp<uint32_t>>                     &patches,
                                  const stl_vector_mp<stl_vector_mp<uint32_t>>                     &components,
                                  const stl_vector_mp<uint32_t>                                    &component_of_patch,
                                  const stl_vector_mp<uint32_t>                                    &next_vert,
                                  // extremal edge of component i is stored at position [2*i], [2*i+1]
                                  stl_vector_mp<uint32_t>                                          &extremal_edge_of_component,
                                  // store an iso-vert index on edge (v, v_next), None means there is no such iso-vert
                                  stl_vector_mp<uint32_t>                                          &iso_vert_on_v_v_next,
                                  // map: (tet_id, tet_face_id) --> iso_face_id
                                  flat_hash_map_mp<face_header_t, uint32_t>                        &iso_face_id_of_tet_face,
                                  // map: (tet_id, tet_vert_id) --> (iso_vert_id, component_id)
                                  flat_hash_map_mp<simplified_vertex_header_t, component_header_t> &iso_vId_compId_of_tet_vert);

// compute the order of iso-vertices on a tet edge v->u, v,u in {0,1,2,3}
// return a list of sorted vertex indices {v_id, i1, i2, ..., u_id}
ISNP_API void compute_edge_intersection_order(const arrangement_t     &tet_cut_result,
                                              uint32_t                 v,
                                              uint32_t                 u,
                                              stl_vector_mp<uint32_t> &vert_indices);

// find the two faces passing v1 and v2, v1->v2 is part of a tet edge
ISNP_API void compute_passing_face_pair(const arrangement_t &tet_cut_result,
                                        uint32_t             v1,
                                        uint32_t             v2,
                                        face_with_orient_t  &face_orient1,
                                        face_with_orient_t  &face_orient2);

// find the face passing v, v->u is part of a tet edge, and u is a tet vertex
ISNP_API void compute_passing_face(const arrangement_t &tet_cut_result,
                                   uint32_t             v,
                                   uint32_t             u,
                                   face_with_orient_t  &face_orient);

// point (x,y,z): dictionary order
inline bool point_xyz_less(const raw_point_t &p, const raw_point_t &q)
{
    return std::lexicographical_compare(p.begin(), p.end(), q.begin(), q.end());
}