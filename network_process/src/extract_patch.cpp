#include <container/hashmap.hpp>
#include <extract_patch.h>

void extract_iso_mesh(size_t                                    num_1_func,
                      size_t                                    num_2_func,
                      size_t                                    num_more_func,
                      const std::vector<Arrangement<3>>&        cut_results,
                      const std::vector<size_t>&                cut_result_index,
                      const std::vector<size_t>&                func_in_tet,
                      const std::vector<size_t>&                start_index_of_tet,
                      const std::vector<std::array<size_t, 4>>& tets,
                      std::vector<IsoVert>&                     iso_verts,
                      std::vector<PolygonFace>&                 iso_faces)
{
    size_t n_tets       = tets.size();
    // estimate number of iso-verts and iso-faces
    size_t max_num_face = num_1_func + 4 * num_2_func + 8 * num_more_func;
    size_t max_num_vert = max_num_face;
    iso_verts.reserve(max_num_vert);
    iso_faces.reserve(max_num_face);
    // hash table for vertices on the boundary of tetrahedron
    absl::flat_hash_map<size_t, size_t>                vert_on_tetVert;
    absl::flat_hash_map<std::array<size_t, 3>, size_t> vert_on_tetEdge;
    vert_on_tetEdge.reserve(num_1_func + 3 * num_2_func + num_more_func);
    absl::flat_hash_map<std::array<size_t, 5>, size_t> vert_on_tetFace;
    vert_on_tetFace.reserve(num_2_func + 6 * num_more_func);
    // hash table for faces on the boundary of tetrahedron
    absl::flat_hash_map<std::array<size_t, 3>, size_t> face_on_tetFace;
    //
    std::vector<bool>                                  is_iso_vert;
    is_iso_vert.reserve(8);
    std::vector<bool> is_iso_face;
    is_iso_face.reserve(9);
    std::vector<size_t> iso_vId_of_vert;
    iso_vId_of_vert.reserve(8);
    std::vector<size_t> face_verts;
    face_verts.reserve(4);
    std::array<size_t, 3> key3;
    std::array<size_t, 5> key5;
    std::array<bool, 4>   used_pId;
    std::array<size_t, 2> vIds2;
    std::array<size_t, 3> vIds3;
    std::array<size_t, 3> implicit_pIds;
    std::array<size_t, 3> bndry_pIds;
    //
    for (size_t i = 0; i < n_tets; i++) {
        if (cut_result_index[i] != Arrangement<3>::None) {
            const auto& arrangement = cut_results[cut_result_index[i]];
            const auto& vertices    = arrangement.vertices;
            const auto& faces       = arrangement.faces;
            auto        start_index = start_index_of_tet[i];
            auto        num_func    = start_index_of_tet[i + 1] - start_index;
            // find vertices and faces on isosurface
            is_iso_vert.clear();
            for (int j = 0; j < vertices.size(); ++j) { is_iso_vert.push_back(false); }
            is_iso_face.clear();
            if (arrangement.unique_planes.empty()) { // all planes are unique
                for (const auto& face : faces) {
                    is_iso_face.push_back(false);
                    if (face.supporting_plane > 3) { // plane 0,1,2,3 are tet boundaries
                        is_iso_face.back() = true;
                        for (const auto& vid : face.vertices) { is_iso_vert[vid] = true; }
                    }
                }
            } else {
                for (const auto& face : faces) {
                    is_iso_face.push_back(false);
                    auto pid = face.supporting_plane;
                    auto uid = arrangement.unique_plane_indices[pid];
                    for (const auto& plane_id : arrangement.unique_planes[uid]) {
                        if (plane_id > 3) { // plane 0,1,2,3 are tet boundaries
                            //                        is_iso_face[j] = true;
                            is_iso_face.back() = true;
                            for (const auto& vid : face.vertices) { is_iso_vert[vid] = true; }
                            break;
                        }
                    }
                }
            }
            // map: local vert index --> iso-vert index
            iso_vId_of_vert.clear();
            // create iso-vertices
            for (size_t j = 0; j < vertices.size(); j++) {
                iso_vId_of_vert.push_back(Arrangement<3>::None);
                if (is_iso_vert[j]) {
                    size_t      num_bndry_planes = 0;
                    size_t      num_impl_planes  = 0;
                    const auto& vertex           = vertices[j];
                    // vertex.size() == 3
                    for (size_t k = 0; k < 3; k++) {
                        if (vertex[k] > 3) { // plane 0,1,2,3 are tet boundaries
                            implicit_pIds[num_impl_planes] = func_in_tet[vertex[k] - 4 + start_index];
                            ++num_impl_planes;
                        } else {
                            bndry_pIds[num_bndry_planes] = vertex[k];
                            ++num_bndry_planes;
                        }
                    }
                    switch (num_bndry_planes) {
                        case 2: // on tet edge
                        {
                            used_pId[0]             = false;
                            used_pId[1]             = false;
                            used_pId[2]             = false;
                            used_pId[3]             = false;
                            used_pId[bndry_pIds[0]] = true;
                            used_pId[bndry_pIds[1]] = true;
                            //                        std::array<size_t, 2> vIds;
                            size_t num_vIds         = 0;
                            for (size_t k = 0; k < 4; k++) {
                                if (!used_pId[k]) {
                                    vIds2[num_vIds] = tets[i][k];
                                    ++num_vIds;
                                }
                            }
                            size_t vId1 = vIds2[0];
                            size_t vId2 = vIds2[1];
                            if (vId1 > vId2) {
                                size_t tmp = vId1;
                                vId1       = vId2;
                                vId2       = tmp;
                            }
                            key3[0]            = vId1;
                            key3[1]            = vId2;
                            key3[2]            = implicit_pIds[0];
                            auto iter_inserted = vert_on_tetEdge.try_emplace(key3, iso_verts.size());
                            if (iter_inserted.second) {
                                iso_verts.emplace_back();
                                auto& iso_vert                   = iso_verts.back();
                                iso_vert.tet_index               = i;
                                iso_vert.tet_vert_index          = j;
                                iso_vert.simplex_size            = 2;
                                iso_vert.simplex_vert_indices[0] = vId1;
                                iso_vert.simplex_vert_indices[1] = vId2;
                                iso_vert.func_indices[0]         = implicit_pIds[0];
                            }
                            iso_vId_of_vert.back() = iter_inserted.first->second;
                            break;
                        }
                        case 1: // on tet face
                        {
                            size_t pId      = bndry_pIds[0];
                            size_t num_vIds = 0;
                            for (size_t k = 0; k < 4; k++) {
                                if (k != pId) {
                                    vIds3[num_vIds] = tets[i][k];
                                    ++num_vIds;
                                }
                            }
                            std::sort(vIds3.begin(), vIds3.end());
                            key5[0]            = vIds3[0];
                            key5[1]            = vIds3[1];
                            key5[2]            = vIds3[2];
                            key5[3]            = implicit_pIds[0];
                            key5[4]            = implicit_pIds[1];
                            auto iter_inserted = vert_on_tetFace.try_emplace(key5, iso_verts.size());
                            if (iter_inserted.second) {
                                iso_verts.emplace_back();
                                auto& iso_vert                   = iso_verts.back();
                                iso_vert.tet_index               = i;
                                iso_vert.tet_vert_index          = j;
                                iso_vert.simplex_size            = 3;
                                iso_vert.simplex_vert_indices[0] = vIds3[0];
                                iso_vert.simplex_vert_indices[1] = vIds3[1];
                                iso_vert.simplex_vert_indices[2] = vIds3[2];
                                iso_vert.func_indices[0]         = implicit_pIds[0];
                                iso_vert.func_indices[1]         = implicit_pIds[1];
                            }
                            iso_vId_of_vert.back() = iter_inserted.first->second;
                            break;
                        }
                        case 0: // in tet cell
                        {
                            iso_vId_of_vert.back() = iso_verts.size();
                            iso_verts.emplace_back();
                            auto& iso_vert                = iso_verts.back();
                            iso_vert.tet_index            = i;
                            iso_vert.tet_vert_index       = j;
                            iso_vert.simplex_size         = 4;
                            iso_vert.simplex_vert_indices = tets[i];
                            iso_vert.func_indices         = implicit_pIds;
                            break;
                        }
                        case 3: // on tet vertex
                        {
                            used_pId[0] = false;
                            used_pId[1] = false;
                            used_pId[2] = false;
                            used_pId[3] = false;
                            for (const auto& pId : bndry_pIds) { used_pId[pId] = true; }
                            size_t vId;
                            for (size_t k = 0; k < 4; k++) {
                                if (!used_pId[k]) {
                                    vId = k;
                                    break;
                                }
                            }
                            auto key           = tets[i][vId];
                            auto iter_inserted = vert_on_tetVert.try_emplace(key, iso_verts.size());
                            if (iter_inserted.second) {
                                iso_verts.emplace_back();
                                auto& iso_vert                   = iso_verts.back();
                                iso_vert.tet_index               = i;
                                iso_vert.tet_vert_index          = j;
                                iso_vert.simplex_size            = 1;
                                iso_vert.simplex_vert_indices[0] = tets[i][vId];
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
            for (size_t j = 0; j < faces.size(); j++) {
                if (is_iso_face[j]) {
                    face_verts.clear();
                    for (unsigned long vId : faces[j].vertices) { face_verts.push_back(iso_vId_of_vert[vId]); }
                    //
                    // face is on tet boundary if face.negative_cell is NONE
                    bool face_on_tet_boundary = (faces[j].negative_cell == Arrangement<3>::None);
                    //
                    if (face_on_tet_boundary) {
                        compute_iso_face_key(face_verts, key3);
                        auto iter_inserted = face_on_tetFace.try_emplace(key3, iso_faces.size());
                        if (iter_inserted.second) {
                            iso_faces.emplace_back();
                            iso_faces.back().vert_indices = face_verts;
                            iso_faces.back().tet_face_indices.emplace_back(i, j);
                            iso_faces.back().func_index.first = func_in_tet[faces[j].supporting_plane - 4 + start_index];
                        } else { // iso_face inserted before
                            size_t iso_face_id = (iter_inserted.first)->second;
                            iso_faces[iso_face_id].tet_face_indices.emplace_back(i, j);
                        }
                    } else { // face not on tet boundary
                        iso_faces.emplace_back();
                        iso_faces.back().vert_indices = face_verts;
                        iso_faces.back().tet_face_indices.emplace_back(i, j);
                        iso_faces.back().func_index.first = func_in_tet[faces[j].supporting_plane - 4 + start_index];
                    }
                }
            }
        }
    }
    //
}

void extract_iso_mesh(size_t                                    num_1_func,
                      size_t                                    num_2_func,
                      size_t                                    num_more_func,
                      const std::vector<Arrangement<3>>&        cut_results,
                      const std::vector<size_t>&                cut_result_index,
                      const std::vector<size_t>&                func_in_tet,
                      const std::vector<size_t>&                start_index_of_tet,
                      const std::vector<std::array<size_t, 4>>& tets,
                      std::vector<IsoVert>&                     iso_verts,
                      std::vector<PolygonFace>&                 iso_faces,
                      std::vector<long long>&                   global_vId_of_tet_vert,
                      std::vector<size_t>&                      global_vId_start_index_of_tet,
                      std::vector<size_t>&                      iso_fId_of_tet_face,
                      std::vector<size_t>&                      iso_fId_start_index_of_tet)
{
    size_t n_tets         = tets.size();
    // get total number of verts and faces
    size_t total_num_vert = 0;
    size_t total_num_face = 0;
    for (const auto& arrangement : cut_results) {
        total_num_vert += arrangement.vertices.size();
        total_num_face += arrangement.faces.size();
    }
    global_vId_of_tet_vert.reserve(total_num_vert);
    global_vId_start_index_of_tet.reserve(n_tets + 1);
    global_vId_start_index_of_tet.push_back(0);
    iso_fId_of_tet_face.reserve(total_num_face);
    iso_fId_start_index_of_tet.reserve(n_tets + 1);
    iso_fId_start_index_of_tet.push_back(0);
    // estimate number of iso-verts and iso-faces
    size_t max_num_face = num_1_func + 4 * num_2_func + 8 * num_more_func;
    size_t max_num_vert = max_num_face;
    iso_verts.reserve(max_num_vert);
    iso_faces.reserve(max_num_face);
    // hash table for vertices on the boundary of tetrahedron
    absl::flat_hash_map<size_t, size_t>                vert_on_tetVert;
    absl::flat_hash_map<std::array<size_t, 3>, size_t> vert_on_tetEdge;
    vert_on_tetEdge.reserve(num_1_func + 3 * num_2_func + num_more_func);
    absl::flat_hash_map<std::array<size_t, 5>, size_t> vert_on_tetFace;
    vert_on_tetFace.reserve(num_2_func + 6 * num_more_func);
    // hash table for faces on the boundary of tetrahedron
    absl::flat_hash_map<std::array<size_t, 3>, size_t> face_on_tetFace;
    //
    std::vector<bool>                                  is_iso_vert;
    is_iso_vert.reserve(8);
    std::vector<bool> is_iso_face;
    is_iso_face.reserve(9);
    std::vector<size_t> iso_vId_of_vert;
    iso_vId_of_vert.reserve(8);
    std::vector<size_t> face_verts;
    face_verts.reserve(4);
    std::array<size_t, 3> key3;
    std::array<size_t, 5> key5;
    std::array<bool, 4>   used_pId;
    std::array<size_t, 2> vIds2;
    std::array<size_t, 3> vIds3;
    std::array<size_t, 3> implicit_pIds;
    std::array<size_t, 3> bndry_pIds;
    //
    for (size_t i = 0; i < n_tets; i++) {
        if (cut_result_index[i] == Arrangement<3>::None) {
            global_vId_start_index_of_tet.push_back(global_vId_of_tet_vert.size());
            iso_fId_start_index_of_tet.push_back(iso_fId_of_tet_face.size());
        } else {
            const auto& arrangement = cut_results[cut_result_index[i]];
            const auto& vertices    = arrangement.vertices;
            const auto& faces       = arrangement.faces;
            auto        start_index = start_index_of_tet[i];
            auto        num_func    = start_index_of_tet[i + 1] - start_index;
            // find vertices and faces on isosurface
            is_iso_vert.clear();
            for (int j = 0; j < vertices.size(); ++j) { is_iso_vert.push_back(false); }
            is_iso_face.clear();
            if (arrangement.unique_planes.empty()) { // all planes are unique
                for (const auto& face : faces) {
                    is_iso_face.push_back(false);
                    if (face.supporting_plane > 3) { // plane 0,1,2,3 are tet boundaries
                        is_iso_face.back() = true;
                        for (const auto& vid : face.vertices) { is_iso_vert[vid] = true; }
                    }
                }
            } else {
                for (const auto& face : faces) {
                    is_iso_face.push_back(false);
                    auto uid = arrangement.unique_plane_indices[face.supporting_plane];
                    for (const auto& plane_id : arrangement.unique_planes[uid]) {
                        if (plane_id > 3) { // plane 0,1,2,3 are tet boundaries
                            //                        is_iso_face[j] = true;
                            is_iso_face.back() = true;
                            for (const auto& vid : face.vertices) { is_iso_vert[vid] = true; }
                            break;
                        }
                    }
                }
            }
            // map: local vert index --> iso-vert index
            iso_vId_of_vert.clear();
            // create iso-vertices
            for (size_t j = 0; j < vertices.size(); j++) {
                size_t      num_bndry_planes = 0;
                size_t      num_impl_planes  = 0;
                const auto& vertex           = vertices[j];
                // vertex.size() == 3
                for (size_t k = 0; k < 3; k++) {
                    if (vertex[k] > 3) { // plane 0,1,2,3 are tet boundaries
                        implicit_pIds[num_impl_planes] = func_in_tet[vertex[k] - 4 + start_index];
                        ++num_impl_planes;
                    } else {
                        bndry_pIds[num_bndry_planes] = vertex[k];
                        ++num_bndry_planes;
                    }
                }
                //
                if (!is_iso_vert[j]) { // vert[j] must be tet vertex
                    used_pId[0] = false;
                    used_pId[1] = false;
                    used_pId[2] = false;
                    used_pId[3] = false;
                    for (const auto& pId : bndry_pIds) { used_pId[pId] = true; }
                    size_t vId;
                    for (size_t k = 0; k < 4; k++) {
                        if (!used_pId[k]) {
                            vId = k;
                            break;
                        }
                    }
                    global_vId_of_tet_vert.push_back(-tets[i][vId] - 1);
                    iso_vId_of_vert.push_back(Arrangement<3>::None);
                } else {        // iso-vertex
                    switch (num_bndry_planes) {
                        case 2: // on tet edge
                        {
                            used_pId[0]             = false;
                            used_pId[1]             = false;
                            used_pId[2]             = false;
                            used_pId[3]             = false;
                            used_pId[bndry_pIds[0]] = true;
                            used_pId[bndry_pIds[1]] = true;
                            size_t num_vIds         = 0;
                            for (size_t k = 0; k < 4; k++) {
                                if (!used_pId[k]) {
                                    vIds2[num_vIds] = tets[i][k];
                                    ++num_vIds;
                                }
                            }
                            size_t vId1 = vIds2[0];
                            size_t vId2 = vIds2[1];
                            if (vId1 > vId2) {
                                size_t tmp = vId1;
                                vId1       = vId2;
                                vId2       = tmp;
                            }
                            key3[0]            = vId1;
                            key3[1]            = vId2;
                            key3[2]            = implicit_pIds[0];
                            auto iter_inserted = vert_on_tetEdge.try_emplace(key3, iso_verts.size());
                            if (iter_inserted.second) {
                                iso_verts.emplace_back();
                                auto& iso_vert                   = iso_verts.back();
                                iso_vert.tet_index               = i;
                                iso_vert.tet_vert_index          = j;
                                iso_vert.simplex_size            = 2;
                                iso_vert.simplex_vert_indices[0] = vId1;
                                iso_vert.simplex_vert_indices[1] = vId2;
                                iso_vert.func_indices[0]         = implicit_pIds[0];
                            }
                            global_vId_of_tet_vert.push_back(iter_inserted.first->second);
                            iso_vId_of_vert.push_back(iter_inserted.first->second);
                            break;
                        }
                        case 1: // on tet face
                        {
                            size_t pId      = bndry_pIds[0];
                            size_t num_vIds = 0;
                            for (size_t k = 0; k < 4; k++) {
                                if (k != pId) {
                                    vIds3[num_vIds] = tets[i][k];
                                    ++num_vIds;
                                }
                            }
                            std::sort(vIds3.begin(), vIds3.end());
                            key5[0]            = vIds3[0];
                            key5[1]            = vIds3[1];
                            key5[2]            = vIds3[2];
                            key5[3]            = implicit_pIds[0];
                            key5[4]            = implicit_pIds[1];
                            auto iter_inserted = vert_on_tetFace.try_emplace(key5, iso_verts.size());
                            if (iter_inserted.second) {
                                iso_verts.emplace_back();
                                auto& iso_vert                   = iso_verts.back();
                                iso_vert.tet_index               = i;
                                iso_vert.tet_vert_index          = j;
                                iso_vert.simplex_size            = 3;
                                iso_vert.simplex_vert_indices[0] = vIds3[0];
                                iso_vert.simplex_vert_indices[1] = vIds3[1];
                                iso_vert.simplex_vert_indices[2] = vIds3[2];
                                iso_vert.func_indices[0]         = implicit_pIds[0];
                                iso_vert.func_indices[1]         = implicit_pIds[1];
                            }
                            global_vId_of_tet_vert.push_back(iter_inserted.first->second);
                            iso_vId_of_vert.push_back(iter_inserted.first->second);
                            break;
                        }
                        case 0: // in tet cell
                        {
                            global_vId_of_tet_vert.push_back(iso_verts.size());
                            iso_vId_of_vert.push_back(iso_verts.size());
                            iso_verts.emplace_back();
                            auto& iso_vert                = iso_verts.back();
                            iso_vert.tet_index            = i;
                            iso_vert.tet_vert_index       = j;
                            iso_vert.simplex_size         = 4;
                            iso_vert.simplex_vert_indices = tets[i];
                            iso_vert.func_indices         = implicit_pIds;
                            break;
                        }
                        case 3: // on tet vertex
                        {
                            used_pId[0] = false;
                            used_pId[1] = false;
                            used_pId[2] = false;
                            used_pId[3] = false;
                            for (const auto& pId : bndry_pIds) { used_pId[pId] = true; }
                            size_t vId;
                            for (size_t k = 0; k < 4; k++) {
                                if (!used_pId[k]) {
                                    vId = k;
                                    break;
                                }
                            }
                            auto key           = tets[i][vId];
                            auto iter_inserted = vert_on_tetVert.try_emplace(key, iso_verts.size());
                            if (iter_inserted.second) {
                                iso_verts.emplace_back();
                                auto& iso_vert                   = iso_verts.back();
                                iso_vert.tet_index               = i;
                                iso_vert.tet_vert_index          = j;
                                iso_vert.simplex_size            = 1;
                                iso_vert.simplex_vert_indices[0] = tets[i][vId];
                            }
                            global_vId_of_tet_vert.push_back(-key - 1);
                            iso_vId_of_vert.push_back(iter_inserted.first->second);
                            break;
                        }
                        default: break;
                    }
                }
            }
            global_vId_start_index_of_tet.push_back(global_vId_of_tet_vert.size());
            // create iso-faces
            for (size_t j = 0; j < faces.size(); j++) {
                iso_fId_of_tet_face.push_back(Arrangement<3>::None);
                if (is_iso_face[j]) {
                    face_verts.clear();
                    for (unsigned long vId : faces[j].vertices) { face_verts.push_back(iso_vId_of_vert[vId]); }
                    //
                    // face is on tet boundary if face.negative_cell is NONE
                    bool face_on_tet_boundary = (faces[j].negative_cell == Arrangement<3>::None);
                    //
                    if (face_on_tet_boundary) {
                        compute_iso_face_key(face_verts, key3);
                        auto iter_inserted = face_on_tetFace.try_emplace(key3, iso_faces.size());
                        if (iter_inserted.second) {
                            iso_fId_of_tet_face.back() = iso_faces.size();
                            iso_faces.emplace_back();
                            iso_faces.back().vert_indices = face_verts;
                            iso_faces.back().tet_face_indices.emplace_back(i, j);
                        } else { // iso_face inserted before
                            size_t iso_face_id         = (iter_inserted.first)->second;
                            iso_fId_of_tet_face.back() = iso_face_id;
                            iso_faces[iso_face_id].tet_face_indices.emplace_back(i, j);
                        }
                        iso_faces.back().func_index.first = func_in_tet[faces[j].supporting_plane - 4 + start_index];
                    } else { // face not on tet boundary
                        iso_fId_of_tet_face.back() = iso_faces.size();
                        iso_faces.emplace_back();
                        iso_faces.back().vert_indices = face_verts;
                        iso_faces.back().tet_face_indices.emplace_back(i, j);
                        iso_faces.back().func_index.first = func_in_tet[faces[j].supporting_plane - 4 + start_index];
                    }
                }
            }
            iso_fId_start_index_of_tet.push_back(iso_fId_of_tet_face.size());
        }
    }
    //
}