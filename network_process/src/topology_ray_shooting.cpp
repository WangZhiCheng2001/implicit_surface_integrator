#include <topology_ray_shooting.hpp>
#include <patch_connectivity.hpp>

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
                                stl_vector_mp<std::pair<uint32_t, uint32_t>>        &shell_links)
{
    // map: tet vert index --> index of next vert (with smaller (x,y,z))
    stl_vector_mp<uint32_t> next_vert{};
    build_next_vert(tet_mesh, next_vert);

    // find extremal edge for each component
    // extremal edge of component i is stored at position [2*i], [2*i+1]
    stl_vector_mp<uint32_t>                                          extremal_edge_of_component{};
    // store an iso-vert index on edge (v, v_next), Mesh_None means there is no such iso-vert
    stl_vector_mp<uint32_t>                                          iso_vert_on_v_v_next{};
    // map: (tet_id, tet_face_id) --> iso_face_id
    flat_hash_map_mp<face_header_t, uint32_t>                        iso_face_id_of_tet_face{};
    // map: (tet_id, tet_vert_id) --> (iso_vert_id, component_id)
    flat_hash_map_mp<simplified_vertex_header_t, component_header_t> iso_vId_compId_of_tet_vert{};
    find_extremal_edges(tet_mesh.vertices,
                        iso_verts,
                        iso_faces,
                        patches,
                        components,
                        component_of_patch,
                        next_vert,
                        extremal_edge_of_component,
                        iso_vert_on_v_v_next,
                        iso_face_id_of_tet_face,
                        iso_vId_compId_of_tet_vert);


    // topological ray shooting
    {
        shell_links.reserve(components.size());
        stl_vector_mp<uint32_t> sorted_vert_indices_on_edge{};
        sorted_vert_indices_on_edge.reserve(3);
        for (uint32_t i = 0; i < components.size(); ++i) {
            // extremal edge: v1 -> v2
            const auto  extreme_v1     = extremal_edge_of_component[2 * i];
            const auto  extreme_v2     = extremal_edge_of_component[2 * i + 1];
            const auto  iso_vId        = iso_vert_on_v_v_next[extreme_v1];
            const auto  tetId          = iso_verts[iso_vId].header.volume_index;
            const auto &tet_cut_result = *cut_results[tetId].get();
            // get local index of v1 and v2 in the tet
            uint32_t    local_v1, local_v2;
            for (uint32_t j = 0; j < 4; ++j) {
                if (tet_mesh.indices[tetId][j] == extreme_v1) {
                    local_v1 = j;
                } else if (tet_mesh.indices[tetId][j] == extreme_v2) {
                    local_v2 = j;
                }
            }
            // get an ordered list of vertices on edge v1 -> v2
            sorted_vert_indices_on_edge.clear();
            compute_edge_intersection_order(tet_cut_result, local_v1, local_v2, sorted_vert_indices_on_edge);
            // find the vertex v_start on v1->v2
            // 1. on current component
            // 2. nearest to v2
            uint32_t j_start;
            for (uint32_t j = 0; j + 1 < sorted_vert_indices_on_edge.size(); ++j) {
                const auto &iso_vId_compId =
                    iso_vId_compId_of_tet_vert[simplified_vertex_header_t{tetId, sorted_vert_indices_on_edge[j]}];
                if (iso_vId_compId.component_index == i) { j_start = j; }
            }
            if (j_start + 2 < sorted_vert_indices_on_edge.size()) {
                // there is a vert from another component between v_start -> v2
                face_with_orient_t face_orient1, face_orient2;
                compute_passing_face_pair(tet_cut_result,
                                          sorted_vert_indices_on_edge[j_start],
                                          sorted_vert_indices_on_edge[j_start + 1],
                                          face_orient1,
                                          face_orient2);
                const auto iso_fId1 = iso_face_id_of_tet_face[face_header_t{tetId, face_orient1.face_id}];
                const auto iso_fId2 = iso_face_id_of_tet_face[face_header_t{tetId, face_orient2.face_id}];
                const auto shell1   = (face_orient1.orient == 1) ? shell_of_half_patch[2 * patch_of_face[iso_fId1]]
                                                                 : shell_of_half_patch[2 * patch_of_face[iso_fId1] + 1];
                const auto shell2   = (face_orient2.orient == 1) ? shell_of_half_patch[2 * patch_of_face[iso_fId2]]
                                                                 : shell_of_half_patch[2 * patch_of_face[iso_fId2] + 1];
                // link shell1 with shell2
                shell_links.emplace_back(shell1, shell2);
            } else {
                // there is no vert between v_start -> v2
                face_with_orient_t face_orient;
                compute_passing_face(tet_cut_result,
                                     sorted_vert_indices_on_edge[j_start],
                                     sorted_vert_indices_on_edge.back(),
                                     face_orient);
                auto       iso_fId     = iso_face_id_of_tet_face[face_header_t{tetId, face_orient.face_id}];
                const auto shell_start = (face_orient.orient == 1) ? shell_of_half_patch[2 * patch_of_face[iso_fId]]
                                                                   : shell_of_half_patch[2 * patch_of_face[iso_fId] + 1];
                // follow the ray till another iso-vertex or the sink
                auto       v_curr      = extreme_v2;
                while (next_vert[v_curr] != invalid_index && iso_vert_on_v_v_next[v_curr] == invalid_index) {
                    v_curr = next_vert[v_curr];
                }
                if (iso_vert_on_v_v_next[v_curr] != invalid_index) {
                    // reached iso-vert at end of the ray
                    const auto  iso_vId_end        = iso_vert_on_v_v_next[v_curr];
                    const auto  end_tetId          = iso_verts[iso_vId_end].header.volume_index;
                    const auto &end_tet_cut_result = *cut_results[end_tetId].get();
                    auto        v_next             = next_vert[v_curr];
                    // find local vertex indices in the end tetrahedron
                    for (uint32_t j = 0; j < 4; ++j) {
                        if (tet_mesh.indices[end_tetId][j] == v_curr) {
                            local_v1 = j;
                        } else if (tet_mesh.indices[end_tetId][j] == v_next) {
                            local_v2 = j;
                        }
                    }
                    // get an ordered list of vertices on edge v_curr -> v_next
                    sorted_vert_indices_on_edge.clear();
                    compute_edge_intersection_order(end_tet_cut_result, local_v1, local_v2, sorted_vert_indices_on_edge);
                    // find the end shell
                    compute_passing_face(end_tet_cut_result,
                                         sorted_vert_indices_on_edge[1],
                                         sorted_vert_indices_on_edge.front(),
                                         face_orient);
                    iso_fId              = iso_face_id_of_tet_face[face_header_t{end_tetId, face_orient.face_id}];
                    const auto shell_end = (face_orient.orient == 1) ? shell_of_half_patch[2 * patch_of_face[iso_fId]]
                                                                     : shell_of_half_patch[2 * patch_of_face[iso_fId] + 1];
                    // link start shell with end shell
                    shell_links.emplace_back(shell_start, shell_end);
                } else {
                    // next_vert[v_curr] is Mesh_None, v_curr is the sink vertex
                    // link shell_start with the sink
                    shell_links.emplace_back(shell_start, invalid_index);
                }
            }
        }
    }
}

ISNP_API void build_next_vert(const tetrahedron_mesh_t &tet_mesh, stl_vector_mp<uint32_t> &next_vert)
{
    next_vert.resize(tet_mesh.vertices.size(), invalid_index);
    for (const auto &tet : tet_mesh.indices) {
        // find the smallest vertex of tet
        uint32_t min_id = 0;
        for (uint32_t i = 1; i < 4; ++i) {
            if (point_xyz_less(tet_mesh.vertices[tet[i]], tet_mesh.vertices[tet[min_id]])) { min_id = i; }
        }
        uint32_t min_vId = tet[min_id];
        //
        for (uint32_t i = 0; i < 4; ++i) {
            if (i != min_id) { next_vert[tet[i]] = min_vId; }
        }
    }
}

ISNP_API void find_extremal_edges(const stl_vector_mp<raw_point_t>                                 &pts,
                                  const stl_vector_mp<iso_vertex_t>                                &iso_verts,
                                  const stl_vector_mp<polygon_face_t>                              &iso_faces,
                                  const stl_vector_mp<stl_vector_mp<uint32_t>>                     &patches,
                                  const stl_vector_mp<stl_vector_mp<uint32_t>>                     &components,
                                  const stl_vector_mp<uint32_t>                                    &component_of_patch,
                                  const stl_vector_mp<uint32_t>                                    &next_vert,
                                  stl_vector_mp<uint32_t>                                          &extremal_edge_of_component,
                                  stl_vector_mp<uint32_t>                                          &iso_vert_on_v_v_next,
                                  flat_hash_map_mp<face_header_t, uint32_t>                        &iso_face_id_of_tet_face,
                                  flat_hash_map_mp<simplified_vertex_header_t, component_header_t> &iso_vId_compId_of_tet_vert)
{
    extremal_edge_of_component.resize(2 * components.size(), invalid_index);
    iso_vert_on_v_v_next.resize(pts.size(), invalid_index);
    iso_face_id_of_tet_face.reserve(iso_faces.size());
    iso_vId_compId_of_tet_vert.reserve(iso_faces.size() / 2);
    //
    stl_vector_mp<bool> is_iso_vert_visited(iso_verts.size(), false);
    for (uint32_t i = 0; i < patches.size(); ++i) {
        uint32_t component_id = component_of_patch[i];
        auto    &u1           = extremal_edge_of_component[2 * component_id];
        auto    &u2           = extremal_edge_of_component[2 * component_id + 1];
        for (auto fId : patches[i]) {
            for (const auto &tet_face : iso_faces[fId].headers) { iso_face_id_of_tet_face.try_emplace(tet_face, fId); }
            for (const auto vId : iso_faces[fId].vertex_indices) {
                if (!is_iso_vert_visited[vId]) {
                    is_iso_vert_visited[vId] = true;
                    const auto &vert         = iso_verts[vId];
                    if (vert.header.minimal_simplex_flag == 2) { // edge iso-vertex
                        const auto v1 = vert.simplex_vertex_indices[0];
                        const auto v2 = vert.simplex_vertex_indices[1];
                        if (next_vert[v1] == v2) {               // on tree edge v1 -> v2
                            // update extremal edge
                            if (u1 == invalid_index) {
                                u1 = v1;
                                u2 = v2;
                            } else {
                                if (v2 == u2) {
                                    if (point_xyz_less(pts[v1], pts[u1])) { u1 = v1; }
                                } else if (point_xyz_less(pts[v2], pts[u2])) {
                                    u1 = v1;
                                    u2 = v2;
                                }
                            }
                            // record an iso-vert on edge v1 -> v2
                            iso_vert_on_v_v_next[v1] = vId;
                            // fill map
                            iso_vId_compId_of_tet_vert.try_emplace(
                                simplified_vertex_header_t{vert.header.volume_index, vert.header.local_vertex_index},
                                component_header_t{vId, component_id});
                        } else if (next_vert[v2] == v1) { // on tree edge v2 -> v1
                            // update extremal edge
                            if (u1 == invalid_index) {
                                u1 = v2;
                                u2 = v1;
                            } else {
                                if (v1 == u2) {
                                    if (point_xyz_less(pts[v2], pts[u1])) { u1 = v2; }
                                } else if (point_xyz_less(pts[v1], pts[u2])) {
                                    u1 = v2;
                                    u2 = v1;
                                }
                            }
                            // record an iso-vert on v2 -> v1
                            iso_vert_on_v_v_next[v2] = vId;
                            // fill map
                            iso_vId_compId_of_tet_vert.try_emplace(
                                simplified_vertex_header_t{vert.header.volume_index, vert.header.local_vertex_index},
                                component_header_t{vId, component_id});
                        }
                    }
                }
            }
        }
    }
}

ISNP_API void compute_edge_intersection_order(const arrangement_t     &tet_cut_result,
                                              uint32_t                 v,
                                              uint32_t                 u,
                                              stl_vector_mp<uint32_t> &vert_indices)
{
    const auto &vertices = tet_cut_result.vertices;
    const auto &faces    = tet_cut_result.faces;

    std::array<bool, 4> edge_flag{true, true, true, true};
    edge_flag[v] = false;
    edge_flag[u] = false;

    // find vertices on edge v->u, and index of v and u in tet_cut_result.vertices
    uint32_t            v_id, u_id;
    stl_vector_mp<bool> is_on_edge_vu(vertices.size(), false);
    uint32_t            num_vert_in_edge_vu = 0;
    uint32_t            flag_count;
    uint32_t            other_plane;
    for (uint32_t i = 0; i < vertices.size(); ++i) {
        flag_count       = 0;
        other_plane      = invalid_index;
        const auto &vert = vertices[i];
        for (int j = 0; j < 3; ++j) {
            if (vert[j] < 4) { // 0,1,2,3 are tet boundary planes
                if (edge_flag[vert[j]]) {
                    ++flag_count;
                } else {
                    other_plane = vert[j];
                }
            }
        }
        if (flag_count == 2) {
            is_on_edge_vu[i] = true;
            if (other_plane == u) {        // current vertex is v
                v_id = i;
            } else if (other_plane == v) { // current vertex is u
                u_id = i;
            } else {                       // current vertex is in interior of edge v->u
                ++num_vert_in_edge_vu;
            }
        }
    }
    if (num_vert_in_edge_vu == 0) { // no intersection points in edge v->u
        vert_indices.emplace_back(v_id);
        vert_indices.emplace_back(u_id);
        return;
    }

    // find all faces on triangle v->u->w, w is a third tet vertex
    uint32_t pId; // plane index of v->u->w, pick pId != v and pId != u
    for (uint32_t i = 0; i < 4; ++i) {
        if (edge_flag[i]) {
            pId = i;
            break;
        }
    }
    //    std::vector<bool> is_on_tri_vuw(faces.size(), false);
    stl_vector_mp<uint32_t> faces_on_tri_vuw{};
    for (uint32_t i = 0; i < faces.size(); ++i) {
        const auto &face = faces[i];
        if (face.negative_cell == invalid_index // face is on tet boundary
            && face.supporting_plane == pId)
            faces_on_tri_vuw.emplace_back(i);
    }

    // build edge-face connectivity on triangle v->u->w
    // map: edge (v1, v2) --> incident faces (f1, f2), f2 is None if (v1,v2) is on triangle boundary
    flat_hash_map_mp<std::pair<uint32_t, uint32_t>, std::pair<uint32_t, uint32_t>> faces_of_edge{};
    for (const auto fId : faces_on_tri_vuw) {
        const auto &face     = faces[fId];
        const auto  num_vert = face.vertices.size();
        for (uint32_t i = 0; i < num_vert; ++i) {
            const auto i_next  = (i + 1) % num_vert;
            const auto vi      = tet_cut_result.vertices[fId][i];
            const auto vi_next = tet_cut_result.vertices[fId][i_next];
            // add fId to edge (vi, vi_next)
            auto iter_inserted = faces_of_edge.try_emplace(std::make_pair(vi, vi_next), std::make_pair(fId, invalid_index));
            if (!iter_inserted.second) { // inserted before
                iter_inserted.first->second.second = fId;
            }
            // add fId to edge (vi_next, vi)
            iter_inserted = faces_of_edge.try_emplace(std::make_pair(vi_next, vi), std::make_pair(fId, invalid_index));
            if (!iter_inserted.second) { // inserted before
                iter_inserted.first->second.second = fId;
            }
        }
    }

    // find the face on triangle v->u->w:
    // 1. has vertex v
    // 2. has an edge on v->u
    uint32_t f_start;
    for (const auto fId : faces_on_tri_vuw) {
        const auto &face   = faces[fId];
        bool        find_v = false;
        uint32_t    count  = 0;
        for (auto vi : face.vertices) {
            if (vi == v_id) { find_v = true; }
            if (is_on_edge_vu[vi]) { ++count; }
        }
        if (find_v && count == 2) {
            f_start = fId;
            break;
        }
    }

    // trace edge v->u
    vert_indices.reserve(num_vert_in_edge_vu + 2);
    vert_indices.emplace_back(v_id);
    stl_vector_mp<bool>           visited_face(faces.size(), false);
    uint32_t                      v_curr = v_id;
    uint32_t                      f_curr = f_start;
    std::pair<uint32_t, uint32_t> edge_prev;
    std::pair<uint32_t, uint32_t> edge_next;
    std::pair<uint32_t, uint32_t> edge_on_vu;
    while (v_curr != u_id) {
        // clear edge_on_vu
        edge_on_vu.first     = invalid_index;
        edge_on_vu.second    = invalid_index;
        //
        const auto &face     = faces[f_curr];
        const auto  num_vert = face.vertices.size();
        // visit all edges of face, find edge_prev, edge_next and edge_on_vu
        for (uint32_t i = 0; i < num_vert; ++i) {
            const auto i_next  = (i + 1) % num_vert;
            const auto vi      = tet_cut_result.vertices[f_curr][i];
            const auto vi_next = tet_cut_result.vertices[f_curr][i_next];
            if (is_on_edge_vu[vi] && !is_on_edge_vu[vi_next]) {
                auto &two_faces  = faces_of_edge[std::make_pair(vi, vi_next)];
                auto  other_face = (two_faces.first == f_curr) ? two_faces.second : two_faces.first;
                if (vi == v_id || (other_face != invalid_index && visited_face[other_face])) {
                    edge_prev.first  = vi;
                    edge_prev.second = vi_next;
                } else {
                    edge_next.first  = vi;
                    edge_next.second = vi_next;
                }
            } else if (is_on_edge_vu[vi_next] && !is_on_edge_vu[vi]) {
                auto &two_faces  = faces_of_edge[std::make_pair(vi, vi_next)];
                auto  other_face = (two_faces.first == f_curr) ? two_faces.second : two_faces.first;
                if (vi_next == v_id || (other_face != invalid_index && visited_face[other_face])) {
                    edge_prev.first  = vi;
                    edge_prev.second = vi_next;
                } else {
                    edge_next.first  = vi;
                    edge_next.second = vi_next;
                }
            } else if (is_on_edge_vu[vi] && is_on_edge_vu[vi_next]) {
                edge_on_vu.first  = vi;
                edge_on_vu.second = vi_next;
            }
        }
        //
        if (edge_on_vu.first == invalid_index) {
            // no edge of the face is on v->u
            // keep v_curr, update f_curr
            visited_face[f_curr] = true;
            auto &two_faces      = faces_of_edge[edge_next];
            f_curr               = (two_faces.first == f_curr) ? two_faces.second : two_faces.first;
        } else {
            // there is an edge of the face on v->u
            // update v_curr to be the next vert on edge v->u
            v_curr = (edge_on_vu.first == v_curr) ? edge_on_vu.second : edge_on_vu.first;
            vert_indices.emplace_back(v_curr);
            // update f_curr
            visited_face[f_curr] = true;
            auto &two_faces      = faces_of_edge[edge_next];
            f_curr               = (two_faces.first == f_curr) ? two_faces.second : two_faces.first;
        }
    }
}

ISNP_API void compute_passing_face_pair(const arrangement_t &tet_cut_result,
                                        uint32_t             v1,
                                        uint32_t             v2,
                                        face_with_orient_t  &face_orient1,
                                        face_with_orient_t  &face_orient2)
{
    // find a face incident to edge v1 -> v2
    const auto &faces = tet_cut_result.faces;
    uint32_t    incident_face_id;
    bool        found_incident_face = false;
    for (uint32_t i = 0; i < faces.size(); ++i) {
        const auto &face     = faces[i];
        uint32_t    num_vert = static_cast<uint32_t>(face.vertices.size());
        for (uint32_t j = 0; j < num_vert; ++j) {
            uint32_t vId1 = face.vertices[j];
            uint32_t vId2 = face.vertices[(j + 1) % num_vert];
            if ((vId1 == v1 && vId2 == v2) || (vId1 == v2 && vId2 == v1)) {
                incident_face_id    = i;
                found_incident_face = true;
                break;
            }
        }
        if (found_incident_face) { break; }
    }
    // assert: found_incident_face == true
    const auto  cell_id = faces[incident_face_id].positive_cell;
    const auto &cell    = tet_cut_result.cells[cell_id];

    // find the two faces
    // 1. bounding the cell
    // 2. passing vertex v1 or v2
    for (auto fId : cell.faces) {
        const auto &face     = faces[fId];
        bool        found_v1 = false;
        bool        found_v2 = false;
        for (auto vId : face.vertices) {
            if (vId == v1) {
                found_v1 = true;
            } else if (vId == v2) {
                found_v2 = true;
            }
        }
        if (found_v1 && !found_v2) {
            // the current face passes v1 but not v2
            face_orient1.face_id = fId;
            face_orient1.orient  = (face.positive_cell == cell_id) ? 1 : -1;
        } else if (!found_v1 && found_v2) {
            // the current face passes v2 but not v1
            face_orient2.face_id = fId;
            face_orient2.orient  = (face.positive_cell == cell_id) ? 1 : -1;
        }
    }
}

ISNP_API void compute_passing_face(const arrangement_t &tet_cut_result, uint32_t v, uint32_t u, face_with_orient_t &face_orient)
{
    // find a face incident to edge v -> u
    const auto &faces = tet_cut_result.faces;
    uint32_t    incident_face_id;
    bool        found_incident_face = false;
    for (uint32_t i = 0; i < faces.size(); ++i) {
        const auto &face     = faces[i];
        uint32_t    num_vert = static_cast<uint32_t>(face.vertices.size());
        for (uint32_t j = 0; j < num_vert; ++j) {
            uint32_t vId1 = face.vertices[j];
            uint32_t vId2 = face.vertices[(j + 1) % num_vert];
            if ((vId1 == v && vId2 == u) || (vId1 == u && vId2 == v)) {
                incident_face_id    = i;
                found_incident_face = true;
                break;
            }
        }
        if (found_incident_face) { break; }
    }
    // assert: found_incident_face == true
    const auto  cell_id = faces[incident_face_id].positive_cell;
    const auto &cell    = tet_cut_result.cells[cell_id];

    // find the face
    // 1. bounding the cell
    // 2. passing vertex v
    for (auto fId : cell.faces) {
        const auto &face    = faces[fId];
        bool        found_v = false;
        bool        found_u = false;
        for (auto vId : face.vertices) {
            if (vId == v) {
                found_v = true;
            } else if (vId == u) {
                found_u = true;
            }
        }
        if (found_v && !found_u) {
            // the current face passes v but not u
            face_orient.face_id = fId;
            face_orient.orient  = (face.positive_cell == cell_id) ? 1 : -1;
            return;
        }
    }
}