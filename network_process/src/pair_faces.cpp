#include <unordered_set>

#include <extract_patch.hpp>

#include <pair_faces.hpp>
#include "utils/fwd_types.hpp"

ISNP_API void compute_patch_order(const iso_edge_t                                                   &iso_edge,
                                  const stl_vector_mp<tetrahedron_vertex_indices_t>                  &tets,
                                  const stl_vector_mp<iso_vertex_t>                                  &iso_verts,
                                  const stl_vector_mp<polygon_face_t>                                &iso_faces,
                                  const stl_vector_mp<std::shared_ptr<arrangement_t>>                &cut_results,
                                  const stl_vector_mp<uint32_t>                                      &func_in_tet,
                                  const stl_vector_mp<uint32_t>                                      &start_index_of_tet,
                                  const parallel_flat_hash_map_mp<uint32_t, stl_vector_mp<uint32_t>> &incident_tets,
                                  const stl_vector_mp<uint32_t>                                      &patch_of_face_mapping,
                                  stl_vector_mp<small_vector_mp<uint32_t>>                           &half_patch_adj_list,
                                  stl_vector_mp<dynamic_bitset_mp<>>                                 &patch_func_signs)
{
    using unordered_set_mp_of_index_t =
        std::unordered_set<uint32_t, std::hash<uint32_t>, std::equal_to<uint32_t>, ScalableMemoryPoolAllocator<uint32_t>>;

    // collect all iso-faces incident to the iso-edge
    const auto                 &face_edge_indices = iso_edge.headers;
    // collect all tets containing the incident iso-faces
    unordered_set_mp_of_index_t containing_tets{};
    for (const auto &face_edge : face_edge_indices) {
        const auto &tetrahedrons = iso_faces[face_edge.face_index].headers;
        for (const auto &t : tetrahedrons) { containing_tets.emplace(t.volume_index); }
    }
    // pair faces
    if (containing_tets.size() == 1) {
        //        std::cout << ">>>>>>>> iso-edge in tet" << std::endl;
        auto tet_id = *containing_tets.begin();
        pair_patches_in_one_tet(*cut_results[tet_id].get(),
                                iso_faces,
                                iso_edge,
                                patch_of_face_mapping,
                                half_patch_adj_list,
                                patch_func_signs);
    } else {
        const auto  v1          = iso_edge.v1;
        const auto  v2          = iso_edge.v2;
        const auto &iso_v1      = iso_verts[v1];
        const auto &iso_v2      = iso_verts[v2];
        bool        on_tet_edge = false;
        uint32_t    vId1, vId2, vId3;
        if (iso_v1.header.minimal_simplex_flag == 1 && iso_v2.header.minimal_simplex_flag == 1) {
            on_tet_edge = true;
            vId1        = iso_v1.simplex_vertex_indices[0];
            vId2        = iso_v2.simplex_vertex_indices[0];
        } else if (iso_v1.header.minimal_simplex_flag == 2 && iso_v2.header.minimal_simplex_flag == 1) {
            vId1        = iso_v1.simplex_vertex_indices[0];
            vId2        = iso_v1.simplex_vertex_indices[1];
            vId3        = iso_v2.simplex_vertex_indices[0];
            on_tet_edge = (vId3 == vId1 || vId3 == vId2);
        } else if (iso_v1.header.minimal_simplex_flag == 1 && iso_v2.header.minimal_simplex_flag == 2) {
            vId1        = iso_v2.simplex_vertex_indices[0];
            vId2        = iso_v2.simplex_vertex_indices[1];
            vId3        = iso_v1.simplex_vertex_indices[0];
            on_tet_edge = (vId3 == vId1 || vId3 == vId2);
        } else if (iso_v1.header.minimal_simplex_flag == 2 && iso_v2.header.minimal_simplex_flag == 2) {
            vId1        = iso_v1.simplex_vertex_indices[0];
            vId2        = iso_v1.simplex_vertex_indices[1];
            // assume: indices in Iso_Vert::simplex_vertex_indices are sorted
            on_tet_edge = (vId1 == iso_v2.simplex_vertex_indices[0] && vId2 == iso_v2.simplex_vertex_indices[1]);
        }
        if (on_tet_edge) {
            //            std::cout << ">>>>>>>> iso-edge on tet edge" << std::endl;
            // iso_edge lies on tet edge (vId1, vId2)
            // tet vertices vId1, vId2 must be degenerate vertices
            // find all tets incident to edge (vId1, vId2)
            unordered_set_mp_of_index_t incident_tets1(incident_tets.at(vId1).begin(), incident_tets.at(vId1).end());
            stl_vector_mp<uint32_t>     common_tetIds{};
            for (const auto tId : incident_tets.at(vId2)) {
                if (incident_tets1.find(tId) != incident_tets1.end()) { common_tetIds.emplace_back(tId); }
            }
            // pair half-faces
            pair_patches_in_tets(iso_edge,
                                 {vId1, vId2},
                                 common_tetIds,
                                 tets,
                                 iso_faces,
                                 cut_results,
                                 func_in_tet,
                                 start_index_of_tet,
                                 patch_of_face_mapping,
                                 half_patch_adj_list,
                                 patch_func_signs);
        } else {
            //            std::cout << ">>>>>>>> iso-edge on tet face" << std::endl;
            // iso_edge lies on a tet boundary face
            // assert: containing_tets.size() == 2
            const auto                  tet_id1 = *containing_tets.begin();
            const auto                  tet_id2 = *std::prev(containing_tets.end());
            unordered_set_mp_of_index_t tet1_vIds(tets[tet_id1].begin(), tets[tet_id1].end());
            stl_vector_mp<uint32_t>     common_vIds{};
            common_vIds.reserve(3);
            for (const auto vId : tets[tet_id2]) {
                if (tet1_vIds.find(vId) != tet1_vIds.end()) common_vIds.emplace_back(vId);
            }
            // pair half-faces
            pair_patches_in_tets(iso_edge,
                                 common_vIds,
                                 {tet_id1, tet_id2},
                                 tets,
                                 iso_faces,
                                 cut_results,
                                 func_in_tet,
                                 start_index_of_tet,
                                 patch_of_face_mapping,
                                 half_patch_adj_list,
                                 patch_func_signs);
        }
    }
}

// ===============================================================================================

ISNP_API void pair_patches_in_one_tet(const arrangement_t                      &tet_cut_result,
                                      const stl_vector_mp<polygon_face_t>      &iso_faces,
                                      const iso_edge_t                         &iso_edge,
                                      const stl_vector_mp<uint32_t>            &patch_of_face_mapping,
                                      stl_vector_mp<small_vector_mp<uint32_t>> &half_patch_adj_list,
                                      stl_vector_mp<dynamic_bitset_mp<>>       &patch_func_signs)
{
    // find tet faces that are incident to the iso_edge
    stl_vector_mp<bool>     is_incident_faces(tet_cut_result.faces.size(), false);
    // map: tet face id --> iso face id
    stl_vector_mp<uint32_t> iso_face_Id_of_face(tet_cut_result.faces.size(), invalid_index);
    for (const auto &fId_eId_pair : iso_edge.headers) {
        const auto iso_face_id       = fId_eId_pair.face_index;
        const auto face_id           = iso_faces[iso_face_id].headers[0].local_face_index;
        is_incident_faces[face_id]   = true;
        iso_face_Id_of_face[face_id] = iso_face_id;
    }

    // travel around the edge
    stl_vector_mp<bool> visited_cell(tet_cut_result.cells.size(), false);

    struct travel_info_t {
        uint32_t iso_face_id{invalid_index};
        uint32_t face_id{invalid_index};
        int8_t   face_sign{0};

        void clear()
        {
            iso_face_id = invalid_index;
            face_id     = invalid_index;
            face_sign   = 0;
        }

        void move_update(travel_info_t &&other)
        {
            *this           = std::move(other);
            this->face_sign = -this->face_sign;
            other.clear();
        }
    };

    auto travel_func = [&](uint32_t &cell_id, travel_info_t &info1, travel_info_t &info2) {
        while (cell_id != invalid_index && !visited_cell[cell_id]) {
            visited_cell[cell_id] = true;
            // find next face
            for (const auto &fId : tet_cut_result.cells[cell_id].faces) {
                if (is_incident_faces[fId] && fId != info1.face_id) { info2.face_id = fId; }
            }
            if (info2.face_id == invalid_index) {
                // next face is not found
                break;
            } else {
                // get sign of face2 and find next cell
                if (tet_cut_result.faces[info2.face_id].positive_cell == cell_id) {
                    cell_id         = tet_cut_result.faces[info2.face_id].negative_cell;
                    info2.face_sign = 1;
                } else {
                    cell_id         = tet_cut_result.faces[info2.face_id].positive_cell;
                    info2.face_sign = -1;
                }
                // add (face1, face2) to the list of face pairs
                info2.iso_face_id            = iso_face_Id_of_face[info2.face_id];
                const auto half_patch_index1 = info1.face_sign > 0 ? 2 * patch_of_face_mapping[info1.iso_face_id]
                                                                   : 2 * patch_of_face_mapping[info1.iso_face_id] + 1;
                const auto half_patch_index2 = info2.face_sign > 0 ? 2 * patch_of_face_mapping[info2.iso_face_id]
                                                                   : 2 * patch_of_face_mapping[info2.iso_face_id] + 1;
                half_patch_adj_list[half_patch_index1].emplace_back(half_patch_index2);
                half_patch_adj_list[half_patch_index2].emplace_back(half_patch_index1);
                if (info2.face_sign > 0) patch_func_signs[half_patch_index1 >> 1][half_patch_index2 >> 1] = true;
                if (info1.face_sign > 0) patch_func_signs[half_patch_index2 >> 1][half_patch_index1 >> 1] = true;
                // update face1 and clear face2
                info1.move_update(std::move(info2));
            }
        }
    };

    // start from the first incident face, and travel to its positive cell
    travel_info_t info1{iso_edge.headers[0].face_index,
                        iso_faces[iso_edge.headers[0].face_index].headers[0].local_face_index,
                        1};
    travel_info_t info2{};
    auto          cell_id = tet_cut_result.faces[info1.face_id].positive_cell;
    travel_func(cell_id, info1, info2);
    // travel in a different direction
    info1 = {iso_edge.headers[0].face_index,                                        //
             iso_faces[iso_edge.headers[0].face_index].headers[0].local_face_index, //
             -1};
    info2.clear();
    cell_id = tet_cut_result.faces[info1.face_id].negative_cell;
    travel_func(cell_id, info1, info2);
}

// ===============================================================================================

ISNP_API void pair_patches_in_tets(const iso_edge_t                                    &iso_edge,
                                   const stl_vector_mp<uint32_t>                       &containing_simplex,
                                   const stl_vector_mp<uint32_t>                       &containing_tetIds,
                                   const stl_vector_mp<tetrahedron_vertex_indices_t>   &tets,
                                   const stl_vector_mp<polygon_face_t>                 &iso_faces,
                                   const stl_vector_mp<std::shared_ptr<arrangement_t>> &cut_results,
                                   const stl_vector_mp<uint32_t>                       &func_in_tet,
                                   const stl_vector_mp<uint32_t>                       &start_index_of_tet,
                                   const stl_vector_mp<uint32_t>                       &patch_of_face_mapping,
                                   stl_vector_mp<small_vector_mp<uint32_t>>            &half_patch_adj_list,
                                   stl_vector_mp<dynamic_bitset_mp<>>                  &patch_func_signs)
{
    //// pre-processing
    // collect all iso-faces incident to the iso-edge
    // map: (tet_id, tet_face_id) -> iso_face_id
    flat_hash_map_mp<face_header_t, uint32_t> iso_face_Id_of_face{};
    for (const auto &[iso_face_id, _] : iso_edge.headers) {
        for (const auto &t : iso_faces[iso_face_id].headers) { iso_face_Id_of_face[t] = iso_face_id; }
    }
    // find identical tet boundary planes incident to iso-edge
    // map: (tet_Id, tet_plane_Id) -> (oppo_tet_Id, oppo_tet_plane_Id)
    flat_hash_map_mp<face_header_t, face_header_t> identical_tet_planes{};
    if (containing_simplex.size() == 3) {
        // iso_edge contained in a tet boundary triangle
        const auto  vId1 = containing_simplex[0];
        const auto  vId2 = containing_simplex[1];
        const auto  vId3 = containing_simplex[2];
        const auto  tId1 = containing_tetIds[0];
        const auto  tId2 = containing_tetIds[1];
        const auto &tet1 = tets[tId1];
        const auto &tet2 = tets[tId2];
        uint32_t    pId1, pId2;
        uint32_t    vId;
        for (uint32_t i = 0; i < 4; ++i) {
            vId = tet1[i];
            if (vId != vId1 && vId != vId2 && vId != vId3) {
                pId1 = i;
                break;
            }
        }
        for (uint32_t i = 0; i < 4; ++i) {
            vId = tet2[i];
            if (vId != vId1 && vId != vId2 && vId != vId3) {
                pId2 = i;
                break;
            }
        }
        // (tId1, pId1) and (tId2, pId2)
        identical_tet_planes[{tId1, pId1}] = {tId2, pId2};
        identical_tet_planes[{tId2, pId2}] = {tId1, pId1};
    } else {
        // iso_edge contained in a tet edge
        uint32_t                                      vId1 = containing_simplex[0];
        uint32_t                                      vId2 = containing_simplex[1];
        flat_hash_map_mp<pod_key_t<3>, face_header_t> tet_plane_of_tri{};
        uint32_t                                      vId;
        pod_key_t<3>                                  tri;
        for (const auto tId : containing_tetIds) {
            for (uint32_t i = 0; i < 4; ++i) {
                vId = tets[tId][i];
                if (vId != vId1 && vId != vId2) {
                    tri = {tets[tId][(i + 1) % 4], tets[tId][(i + 2) % 4], tets[tId][(i + 3) % 4]};
                    std::sort(tri.begin(), tri.end());
                    auto iter_inserted = tet_plane_of_tri.try_emplace(tri, face_header_t{tId, i});
                    if (!iter_inserted.second) {
                        identical_tet_planes[{tId, i}]                    = iter_inserted.first->second;
                        identical_tet_planes[iter_inserted.first->second] = {tId, i};
                    }
                }
            }
        }
    }
    // find identical faces (tet_id, tet_face_id) on tet boundary planes incident to iso-edge
    // map: (tet_Id, tet_face_Id) -> (oppo_tet_Id, oppo_tet_face_Id)
    flat_hash_map_mp<face_header_t, face_header_t> opposite_face{};
    // map: (tet_Id, tet_vert_Id) -> boundary vert Id
    flat_hash_map_mp<face_header_t, uint32_t>      boundary_vert_id{};
    uint32_t                                       num_boundary_vert = 0;
    // hash table for vertices on the boundary of tetrahedron
    flat_hash_map_mp<uint32_t, uint32_t>           vert_on_tetVert{};
    flat_hash_map_mp<pod_key_t<3>, uint32_t>       vert_on_tetEdge{};
    flat_hash_map_mp<pod_key_t<5>, uint32_t>       vert_on_tetFace{};
    // hash table for faces on the boundary of tetrahedron
    // map: (i,j,k) -> (tet_Id, tet_face_Id)
    flat_hash_map_mp<pod_key_t<3>, face_header_t>  face_on_tetFace{};
    // auxiliary data
    stl_vector_mp<bool>                            is_boundary_vert{};
    stl_vector_mp<bool>                            is_boundary_face{};
    stl_vector_mp<uint32_t>                        boundary_vId_of_vert{};
    stl_vector_mp<uint32_t>                        face_verts{};
    pod_key_t<3>                                   key3;
    pod_key_t<5>                                   key5;
    std::array<bool, 4>                            used_pId;
    std::array<uint32_t, 2>                        vIds2;
    std::array<uint32_t, 3>                        vIds3;
    std::array<uint32_t, 3>                        implicit_pIds;
    std::array<uint32_t, 3>                        boundary_pIds;
    for (const auto i : containing_tetIds) {
        if (cut_results[i]) {
            // empty tet i
            for (uint32_t fi = 0; fi < 4; ++fi) {
                if (identical_tet_planes.find(face_header_t{i, fi}) != identical_tet_planes.end()) {
                    // face fi lies on a tet boundary incident to iso-edge
                    pod_key_t<3> bounary_face_verts;
                    for (uint32_t j = 0; j < 3; ++j) {
                        auto key           = tets[i][(fi + j + 1) % 4];
                        auto iter_inserted = vert_on_tetVert.try_emplace(key, num_boundary_vert);
                        if (iter_inserted.second) { num_boundary_vert++; }
                        bounary_face_verts[j] = iter_inserted.first->second;
                    }
                    std::sort(bounary_face_verts.begin(), bounary_face_verts.end());
                    auto iter_inserted = face_on_tetFace.try_emplace(bounary_face_verts, face_header_t{i, fi});
                    if (!iter_inserted.second) {
                        // face inserted before
                        opposite_face[iter_inserted.first->second] = {i, fi};
                        opposite_face[{i, fi}]                     = iter_inserted.first->second;
                    }
                }
            }
        } else {
            // non-empty tet i
            const auto &arrangement = *cut_results[i].get();
            const auto &vertices    = arrangement.vertices;
            const auto &faces       = arrangement.faces;
            auto        start_index = start_index_of_tet[i];
            auto        num_func    = start_index_of_tet[i + 1] - start_index;
            // find vertices and faces on tet boundary incident to iso-edge
            is_boundary_vert.assign(arrangement.vertices.size(), false);
            is_boundary_face.clear();
            is_boundary_face.reserve(faces.size());
            for (const auto &face : faces) {
                is_boundary_face.emplace_back(false);
                if (face.supporting_plane < 4
                    && identical_tet_planes.find({i, face.supporting_plane}) != identical_tet_planes.end()) {
                    is_boundary_face.back() = true;
                    for (const auto &vid : face.vertices) { is_boundary_vert[vid] = true; }
                }
            }
            // map: local vert index --> boundary vert index
            boundary_vId_of_vert.clear();
            boundary_vId_of_vert.reserve(vertices.size());
            // create boundary vertices
            for (uint32_t j = 0; j < vertices.size(); j++) {
                boundary_vId_of_vert.emplace_back(invalid_index);
                if (is_boundary_vert[j]) {
                    uint32_t    num_boundary_planes = 0;
                    uint32_t    num_impl_planes     = 0;
                    const auto &vertex              = vertices[j];
                    // vertex.size() == 3
                    for (uint32_t k = 0; k < 3; k++) {
                        if (vertex[k] > 3) { // plane 0,1,2,3 are tet boundaries
                            implicit_pIds[num_impl_planes] = func_in_tet[vertex[k] - 4 + start_index];
                            ++num_impl_planes;
                        } else {
                            boundary_pIds[num_boundary_planes] = vertex[k];
                            ++num_boundary_planes;
                        }
                    }
                    switch (num_boundary_planes) {
                        case 2: // on tet edge
                        {
                            used_pId.fill(false);
                            used_pId[boundary_pIds[0]] = true;
                            used_pId[boundary_pIds[1]] = true;
                            //                        std::array<uint32_t, 2> vIds;
                            uint32_t num_vIds          = 0;
                            for (uint32_t k = 0; k < 4; k++) {
                                if (!used_pId[k]) {
                                    vIds2[num_vIds] = tets[i][k];
                                    ++num_vIds;
                                }
                            }
                            uint32_t vId1 = vIds2[0];
                            uint32_t vId2 = vIds2[1];
                            if (vId1 > vId2) std::swap(vId1, vId2);
                            key3[0]            = vId1;
                            key3[1]            = vId2;
                            key3[2]            = implicit_pIds[0];
                            auto iter_inserted = vert_on_tetEdge.try_emplace(key3, num_boundary_vert);
                            if (iter_inserted.second) { num_boundary_vert++; }
                            boundary_vId_of_vert.back() = iter_inserted.first->second;
                            break;
                        }
                        case 1: // on tet face
                        {
                            uint32_t pId      = boundary_pIds[0];
                            uint32_t num_vIds = 0;
                            for (uint32_t k = 0; k < 4; k++) {
                                if (k != pId) {
                                    vIds3[num_vIds] = tets[i][k];
                                    ++num_vIds;
                                }
                            }
                            std::sort(vIds3.begin(), vIds3.end());
                            key5               = {vIds3[0], vIds3[1], vIds3[2], implicit_pIds[0], implicit_pIds[1]};
                            auto iter_inserted = vert_on_tetFace.try_emplace(key5, num_boundary_vert);
                            if (iter_inserted.second) { num_boundary_vert++; }
                            boundary_vId_of_vert.back() = iter_inserted.first->second;
                            break;
                        }
                        case 3: // on tet vertex
                        {
                            used_pId.fill(false);
                            for (const auto &pId : boundary_pIds) { used_pId[pId] = true; }
                            uint32_t vId;
                            for (uint32_t k = 0; k < 4; k++) {
                                if (!used_pId[k]) {
                                    vId = k;
                                    break;
                                }
                            }
                            auto key           = tets[i][vId];
                            auto iter_inserted = vert_on_tetVert.try_emplace(key, num_boundary_vert);
                            if (iter_inserted.second) { num_boundary_vert++; }
                            boundary_vId_of_vert.back() = iter_inserted.first->second;
                            break;
                        }
                        default: break;
                    }
                }
            }
            // pair boundary faces
            for (uint32_t j = 0; j < faces.size(); ++j) {
                if (is_boundary_face[j]) {
                    face_verts.clear();
                    for (auto vId : faces[j].vertices) { face_verts.emplace_back(boundary_vId_of_vert[vId]); }
                    compute_iso_face_key(face_verts, key3);
                    auto iter_inserted = face_on_tetFace.try_emplace(key3, face_header_t{i, j});
                    if (!iter_inserted.second) {
                        // face inserted before, pair the two faces
                        opposite_face[iter_inserted.first->second] = {i, j};
                        opposite_face[{i, j}]                      = iter_inserted.first->second;
                    }
                }
            }
        }
    }

    // find the half-iso-face of the local face in tet with given orientation
    // the orientation of an iso-face is defined by the smallest-index implicit function passing the iso-face
    auto get_half_iso_face = [&](face_header_t tet_face, int8_t orient, uint32_t &iso_face_id, int8_t &iso_orient) {
        iso_face_id              = iso_face_Id_of_face[tet_face];
        const auto &cell_complex = *cut_results[tet_face.volume_index].get();
        const auto &faces        = cell_complex.faces;
        auto        supp_pId     = faces[tet_face.local_face_index].supporting_plane;
        if (supp_pId > 3) { // plane 0,1,2,3 are tet boundary planes
            // supporting plane is not a tet boundary plane
            // assume: supporting plane is the iso-surface of smallest-index implicit function
            iso_orient = orient;
        } else {
            // supporting plane is a tet boundary plane, must be duplicate planes
            const auto uid     = cell_complex.unique_plane_indices[supp_pId];
            // find the smallest-index non-boundary plane
            uint32_t   min_pId = std::numeric_limits<uint32_t>::max();
            for (auto pId : cell_complex.unique_planes[uid]) {
                if (pId > 3 && pId < min_pId) { min_pId = pId; }
            }
            // orient is the orientation of supporting plane
            // flip orientation if smallest-index non-boundary plane has different orientation
            if (cell_complex.unique_plane_orientations[min_pId] != cell_complex.unique_plane_orientations[supp_pId]) {
                iso_orient = -orient;
            } else {
                iso_orient = orient;
            }
        }
    };

    // pair (tet_id, tet_face_id)
    auto find_next = [&](face_header_t face, int8_t orient, face_header_t &face_next, int8_t &orient_next, auto &&find_next) {
        const auto tet_id      = face.volume_index;
        const auto tet_face_id = face.local_face_index;
        if (!cut_results[tet_id]) {
            // empty tet
            if (orient == 1) { // Positive side: the tet
                for (uint32_t fi = 0; fi < 4; ++fi) {
                    if (fi != tet_face_id && opposite_face.find({tet_id, fi}) != opposite_face.end()) {
                        face_next   = {tet_id, fi};
                        orient_next = 1;
                        return;
                    }
                }
            } else { // Negative side: None
                find_next(opposite_face[face], 1, face_next, orient_next, find_next);
            }
        } else {
            // non-empty tet
            const auto &cell_complex = *cut_results[tet_id].get();
            uint32_t    cell_id =
                (orient == 1 ? cell_complex.faces[tet_face_id].positive_cell : cell_complex.faces[tet_face_id].negative_cell);
            if (cell_id != invalid_index) {
                for (auto fi : cell_complex.cells[cell_id].faces) {
                    if (fi != tet_face_id
                        && (iso_face_Id_of_face.find({tet_id, fi}) != iso_face_Id_of_face.end()
                            || opposite_face.find({tet_id, fi}) != opposite_face.end())) {
                        face_next = {tet_id, fi};
                        break;
                    }
                }
                orient_next = cell_complex.faces[face_next.local_face_index].positive_cell == cell_id ? 1 : -1;
            } else {
                // cell is None, so the face lies on tet boundary
                find_next(opposite_face[face], 1, face_next, orient_next, find_next);
            }
        }
    };


    //// face pairing algorithm
    const auto    iso_face_0  = iso_edge.headers[0].face_index;
    // pair (tet_id, tet_face_id)
    auto          face_curr   = iso_faces[iso_face_0].headers[0];
    int8_t        orient_curr = 1; // orientation: 1 for Positive, -1 for Negative
    face_header_t face_next;
    int8_t        orient_next, iso_orient_curr, iso_orient_next;
    uint32_t      iso_face_curr = iso_face_0;
    uint32_t      iso_face_next = invalid_index;
    while (iso_face_next != iso_face_0) {
        find_next(face_curr, orient_curr, face_next, orient_next, find_next);
        while (iso_face_Id_of_face.find(face_next) == iso_face_Id_of_face.end()) {
            find_next(face_next, -orient_next, face_next, orient_next, find_next);
        }
        get_half_iso_face(face_curr, orient_curr, iso_face_curr, iso_orient_curr);
        get_half_iso_face(face_next, orient_next, iso_face_next, iso_orient_next);
        //
        const auto half_patch_index1 =
            iso_orient_curr > 0 ? 2 * patch_of_face_mapping[iso_face_curr] : 2 * patch_of_face_mapping[iso_face_curr] + 1;
        const auto half_patch_index2 =
            iso_orient_next > 0 ? 2 * patch_of_face_mapping[iso_face_next] : 2 * patch_of_face_mapping[iso_face_next] + 1;
        half_patch_adj_list[half_patch_index1].emplace_back(half_patch_index2);
        half_patch_adj_list[half_patch_index2].emplace_back(half_patch_index1);
        if (iso_orient_next > 0) patch_func_signs[half_patch_index1 >> 1][half_patch_index2 >> 1] = true;
        if (iso_orient_curr > 0) patch_func_signs[half_patch_index2 >> 1][half_patch_index1 >> 1] = true;
        //
        face_curr   = face_next;
        orient_curr = -orient_next;
    }
}