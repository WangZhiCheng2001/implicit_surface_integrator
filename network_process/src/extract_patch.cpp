#include <container/hashmap.hpp>

#include <extract_patch.hpp>
#include "implicit_arrangement.hpp"
#include "utils/fwd_types.hpp"

/// TODO: compress implicit function indices into uint16_t instead of uint32_t
ISNP_API void extract_iso_mesh(uint32_t                                           num_1_func,
                               uint32_t                                           num_2_func,
                               uint32_t                                           num_more_func,
                               const stl_vector_mp<arrangement_t>&                cut_results,
                               const stl_vector_mp<uint32_t>&                     cut_result_index,
                               const stl_vector_mp<uint32_t>&                     func_in_tet,
                               const stl_vector_mp<uint32_t>&                     start_index_of_tet,
                               const stl_vector_mp<tetrahedron_vertex_indices_t>& tets,
                               stl_vector_mp<iso_vertex_t>&                       iso_verts,
                               stl_vector_mp<polygon_face_t>&                     iso_faces)
{
    uint32_t n_tets       = static_cast<uint32_t>(tets.size());
    // estimate number of iso-verts and iso-faces
    uint32_t max_num_face = num_1_func + 4 * num_2_func + 8 * num_more_func;
    uint32_t max_num_vert = max_num_face;
    iso_verts.reserve(max_num_vert);
    iso_faces.reserve(max_num_face);

    // hash table for vertices on the boundary of tetrahedron
    flat_hash_map_mp<uint32_t, uint32_t>     vert_on_tetVert{};
    flat_hash_map_mp<pod_key_t<3>, uint32_t> vert_on_tetEdge{};
    vert_on_tetEdge.reserve(num_1_func + 3 * num_2_func + num_more_func);
    flat_hash_map_mp<pod_key_t<5>, uint32_t> vert_on_tetFace{};
    vert_on_tetFace.reserve(num_2_func + 6 * num_more_func);

    // hash table for faces on the boundary of tetrahedron
    flat_hash_map_mp<pod_key_t<3>, uint32_t> face_on_tetFace{};

    //
    stl_vector_mp<bool> is_iso_vert{};
    is_iso_vert.reserve(8);
    stl_vector_mp<bool> is_iso_face{};
    is_iso_face.reserve(9);
    stl_vector_mp<uint32_t> iso_vId_of_vert{};
    iso_vId_of_vert.reserve(8);
    stl_vector_mp<uint32_t> face_verts{};
    face_verts.reserve(4);
    pod_key_t<3>            key3;
    pod_key_t<5>            key5;
    std::array<bool, 4>     used_pId;
    std::array<uint32_t, 2> vIds2;
    std::array<uint32_t, 3> vIds3;
    std::array<uint32_t, 3> implicit_pIds;
    std::array<uint32_t, 3> boundary_pIds;

    //
    for (uint32_t i = 0; i < n_tets; i++) {
        if (cut_result_index[i] != invalid_index) {
            const auto& arrangement = cut_results[cut_result_index[i]];
            const auto& vertices    = arrangement.vertices;
            const auto& faces       = arrangement.faces;
            auto        start_index = start_index_of_tet[i];
            auto        num_func    = start_index_of_tet[i + 1] - start_index;

            // find vertices and faces on isosurface
            is_iso_vert.assign(vertices.size(), false);
            is_iso_face.clear();
            is_iso_face.reserve(faces.size());
            if (arrangement.unique_planes.empty()) { // all planes are unique
                for (const auto& face : faces) {
                    is_iso_face.emplace_back(false);
                    if (face.supporting_plane > 3) { // plane 0,1,2,3 are tet boundaries
                        is_iso_face.back() = true;
                        for (const auto& vid : face.vertices) { is_iso_vert[vid] = true; }
                    }
                }
            } else {
                for (const auto& face : faces) {
                    is_iso_face.emplace_back(false);
                    auto pid = face.supporting_plane;
                    auto uid = arrangement.unique_plane_indices[pid];
                    for (const auto& plane_id : arrangement.unique_planes[uid]) {
                        if (plane_id > 3) { // plane 0,1,2,3 are tet boundaries
                            is_iso_face.back() = true;
                            for (const auto& vid : face.vertices) { is_iso_vert[vid] = true; }
                            break;
                        }
                    }
                }
            }

            // map: local vert index --> iso-vert index
            iso_vId_of_vert.clear();
            iso_vId_of_vert.reserve(vertices.size());
            // create iso-vertices
            for (uint32_t j = 0; j < vertices.size(); j++) {
                iso_vId_of_vert.emplace_back(invalid_index);
                if (is_iso_vert[j]) {
                    uint32_t    num_boundary_planes = 0;
                    uint32_t    num_impl_planes     = 0;
                    const auto& vertex              = vertices[j];
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
                            key3               = {vId1, vId2, implicit_pIds[0]};
                            auto iter_inserted = vert_on_tetEdge.try_emplace(key3, static_cast<uint32_t>(iso_verts.size()));
                            if (iter_inserted.second) {
                                auto& iso_vert                     = iso_verts.emplace_back();
                                iso_vert.header                    = {i, j, 2};
                                iso_vert.simplex_vertex_indices    = {vId1, vId2};
                                iso_vert.implicit_function_indices = {implicit_pIds[0]};
                            }
                            iso_vId_of_vert.back() = iter_inserted.first->second;
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
                            auto iter_inserted = vert_on_tetFace.try_emplace(key5, static_cast<uint32_t>(iso_verts.size()));
                            if (iter_inserted.second) {
                                auto& iso_vert                     = iso_verts.emplace_back();
                                iso_vert.header                    = {i, j, 3};
                                iso_vert.simplex_vertex_indices    = {vIds3[0], vIds3[1], vIds3[2]};
                                iso_vert.implicit_function_indices = {implicit_pIds[0], implicit_pIds[1]};
                            }
                            iso_vId_of_vert.back() = iter_inserted.first->second;
                            break;
                        }
                        case 0: // in tet cell
                        {
                            iso_vId_of_vert.back()             = static_cast<uint32_t>(iso_verts.size());
                            auto& iso_vert                     = iso_verts.emplace_back();
                            iso_vert.header                    = {i, j, 4};
                            iso_vert.simplex_vertex_indices    = tets[i];
                            iso_vert.implicit_function_indices = implicit_pIds;
                            break;
                        }
                        case 3: // on tet vertex
                        {
                            used_pId.fill(false);
                            for (const auto& pId : boundary_pIds) { used_pId[pId] = true; }
                            uint32_t vId;
                            for (uint32_t k = 0; k < 4; k++) {
                                if (!used_pId[k]) {
                                    vId = k;
                                    break;
                                }
                            }
                            auto key           = tets[i][vId];
                            auto iter_inserted = vert_on_tetVert.try_emplace(key, static_cast<uint32_t>(iso_verts.size()));
                            if (iter_inserted.second) {
                                auto& iso_vert                     = iso_verts.emplace_back();
                                iso_vert.header                    = {i, j, 1};
                                iso_vert.implicit_function_indices = {tets[i][vId]};
                            }
                            //                        iso_vId_of_vert[j] = iter_inserted.first->second;
                            iso_vId_of_vert.back() = iter_inserted.first->second;
                            break;
                        }
                        default: break;
                    }
                }
            }
            // create iso-faces
            for (uint32_t j = 0; j < faces.size(); j++) {
                if (is_iso_face[j]) {
                    face_verts.clear();
                    for (unsigned long vId : faces[j].vertices) { face_verts.emplace_back(iso_vId_of_vert[vId]); }
                    //
                    // face is on tet boundary if face.negative_cell is NONE
                    bool face_on_tet_boundary = (faces[j].negative_cell == invalid_index);
                    //
                    if (face_on_tet_boundary) {
                        compute_iso_face_key(face_verts, key3);
                        auto iter_inserted = face_on_tetFace.try_emplace(key3, static_cast<uint32_t>(iso_faces.size()));
                        if (iter_inserted.second) {
                            auto& face          = iso_faces.emplace_back();
                            face.vertex_indices = face_verts;
                            face.headers.emplace_back(i, j);
                            face.implicit_function_index = func_in_tet[faces[j].supporting_plane - 4 + start_index];
                        } else { // iso_face inserted before
                            uint32_t iso_face_id = (iter_inserted.first)->second;
                            iso_faces[iso_face_id].headers.emplace_back(i, j);
                        }
                    } else { // face not on tet boundary
                        auto& face          = iso_faces.emplace_back();
                        face.vertex_indices = face_verts;
                        face.headers.emplace_back(i, j);
                        face.implicit_function_index = func_in_tet[faces[j].supporting_plane - 4 + start_index];
                    }
                }
            }
        }
    }
    //
}

ISNP_API void compute_iso_face_key(const stl_vector_mp<uint32_t>& face_verts, pod_key_t<3>& key)
{
    auto     min_vert = face_verts[0];
    uint32_t min_pos  = 0;
    auto     max_vert = face_verts[0];
    for (uint32_t i = 1; i < face_verts.size(); i++) {
        if (face_verts[i] < min_vert) {
            min_vert = face_verts[i];
            min_pos  = i;
        } else if (face_verts[i] > max_vert) {
            max_vert = face_verts[i];
        }
    }
    auto second_min_vert = max_vert + 1;
    for (uint32_t i = 0; i < face_verts.size(); i++) {
        if (i != min_pos && face_verts[i] < second_min_vert) { second_min_vert = face_verts[i]; }
    }
    //
    key[0] = min_vert;
    key[1] = second_min_vert;
    key[2] = max_vert;
}

ISNP_API void compute_iso_vert_xyz(const stl_vector_mp<iso_vertex_t>&       iso_verts,
                                   const Eigen::Ref<const Eigen::MatrixXd>& func_vals,
                                   const stl_vector_mp<raw_point_t>&        pts,
                                   stl_vector_mp<raw_point_t>&              iso_pts)
{
    iso_pts.resize(iso_verts.size());
    std::array<double, 2> b2;
    std::array<double, 3> f1s3;
    std::array<double, 3> f2s3;
    std::array<double, 3> b3;
    std::array<double, 4> f1s4;
    std::array<double, 4> f2s4;
    std::array<double, 4> f3s4;
    std::array<double, 4> b4;
    for (uint32_t i = 0; i < iso_verts.size(); i++) {
        const auto& iso_vert = iso_verts[i];
        switch (iso_vert.header.minimal_simplex_flag) {
            case 2: // on tet edge
            {
                const auto vId1 = iso_vert.simplex_vertex_indices[0];
                const auto vId2 = iso_vert.simplex_vertex_indices[1];
                const auto fId  = iso_vert.implicit_function_indices[0];
                const auto f1   = func_vals(fId, vId1);
                const auto f2   = func_vals(fId, vId2);
                //
                compute_barycentric_coords(f1, f2, b2);
                iso_pts[i][0] = b2[0] * pts[vId1][0] + b2[1] * pts[vId2][0];
                iso_pts[i][1] = b2[0] * pts[vId1][1] + b2[1] * pts[vId2][1];
                iso_pts[i][2] = b2[0] * pts[vId1][2] + b2[1] * pts[vId2][2];
                break;
            }
            case 3: // on tet face
            {
                const auto vId1 = iso_vert.simplex_vertex_indices[0];
                const auto vId2 = iso_vert.simplex_vertex_indices[1];
                const auto vId3 = iso_vert.simplex_vertex_indices[2];
                const auto fId1 = iso_vert.implicit_function_indices[0];
                const auto fId2 = iso_vert.implicit_function_indices[1];
                //
                f1s3[0]         = func_vals(fId1, vId1);
                f1s3[1]         = func_vals(fId1, vId2);
                f1s3[2]         = func_vals(fId1, vId3);
                //
                f2s3[0]         = func_vals(fId2, vId1);
                f2s3[1]         = func_vals(fId2, vId2);
                f2s3[2]         = func_vals(fId2, vId3);
                //
                compute_barycentric_coords(f1s3, f2s3, b3);
                iso_pts[i][0] = b3[0] * pts[vId1][0] + b3[1] * pts[vId2][0] + b3[2] * pts[vId3][0];
                iso_pts[i][1] = b3[0] * pts[vId1][1] + b3[1] * pts[vId2][1] + b3[2] * pts[vId3][1];
                iso_pts[i][2] = b3[0] * pts[vId1][2] + b3[1] * pts[vId2][2] + b3[2] * pts[vId3][2];
                break;
            }
            case 4: // in tet cell
            {
                auto vId1 = iso_vert.simplex_vertex_indices[0];
                auto vId2 = iso_vert.simplex_vertex_indices[1];
                auto vId3 = iso_vert.simplex_vertex_indices[2];
                auto vId4 = iso_vert.simplex_vertex_indices[3];
                auto fId1 = iso_vert.implicit_function_indices[0];
                auto fId2 = iso_vert.implicit_function_indices[1];
                auto fId3 = iso_vert.implicit_function_indices[2];
                f1s4[0]   = func_vals(fId1, vId1);
                f1s4[1]   = func_vals(fId1, vId2);
                f1s4[2]   = func_vals(fId1, vId3);
                f1s4[3]   = func_vals(fId1, vId4);
                //
                f2s4[0]   = func_vals(fId2, vId1);
                f2s4[1]   = func_vals(fId2, vId2);
                f2s4[2]   = func_vals(fId2, vId3);
                f2s4[3]   = func_vals(fId2, vId4);
                //
                f3s4[0]   = func_vals(fId3, vId1);
                f3s4[1]   = func_vals(fId3, vId2);
                f3s4[2]   = func_vals(fId3, vId3);
                f3s4[3]   = func_vals(fId3, vId4);
                //
                compute_barycentric_coords(f1s4, f2s4, f3s4, b4);
                iso_pts[i][0] = b4[0] * pts[vId1][0] + b4[1] * pts[vId2][0] + b4[2] * pts[vId3][0] + b4[3] * pts[vId4][0];
                iso_pts[i][1] = b4[0] * pts[vId1][1] + b4[1] * pts[vId2][1] + b4[2] * pts[vId3][1] + b4[3] * pts[vId4][1];
                iso_pts[i][2] = b4[0] * pts[vId1][2] + b4[1] * pts[vId2][2] + b4[2] * pts[vId3][2] + b4[3] * pts[vId4][2];
                break;
            }
            case 1: // on tet vertex
                iso_pts[i] = pts[iso_vert.simplex_vertex_indices[0]];
                break;
            default: break;
        }
    }
}