#include <queue>

#include <container/hashmap.hpp>

#include <patch_connectivity.hpp>

ISNP_API void compute_patch_edges(const stl_vector_mp<polygon_face_t>&    patch_faces,
                                  stl_vector_mp<stl_vector_mp<uint32_t>>& edges_of_face,
                                  stl_vector_mp<iso_edge_t>&              patch_edges)
{
    uint32_t max_num_edge = 0;
    for (const auto& iso_face : patch_faces) { max_num_edge += static_cast<uint32_t>(iso_face.vertex_indices.size()); }
    patch_edges.reserve(max_num_edge / 2);
    edges_of_face.reserve(patch_faces.size());
    uint32_t                                                  num_iso_edge{};
    // map: (v1, v2) -> iso-edge index
    flat_hash_map_mp<std::pair<uint32_t, uint32_t>, uint32_t> edge_id{};
    for (uint32_t i = 0; i < patch_faces.size(); i++) {
        auto&          face       = patch_faces[i];
        const uint32_t num_edge   = static_cast<uint32_t>(face.vertex_indices.size());
        // emplace at back of edges_of_face a vector<uint32_t> of size num_edge
        auto&          face_edges = edges_of_face.emplace_back(num_edge);
        for (uint32_t j = 0; j < num_edge; j++) {
            auto v1 = face.vertex_indices[j];
            auto v2 = (j + 1 == num_edge) ? face.vertex_indices[0] : face.vertex_indices[j + 1];
            // swap if v1 > v2
            if (v1 > v2) std::swap(v1, v2);
            //
            num_iso_edge       = static_cast<uint32_t>(patch_edges.size());
            auto iter_inserted = edge_id.try_emplace(std::make_pair(v1, v2), num_iso_edge);
            if (iter_inserted.second) { // new iso-edge
                auto& edge = patch_edges.emplace_back();
                edge.v1    = v1;
                edge.v2    = v2;
                edge.headers.emplace_back(i, j);
                face_edges[j] = num_iso_edge;
            } else { // existing iso-edge
                uint32_t eId = iter_inserted.first->second;
                patch_edges[eId].headers.emplace_back(i, j);
                face_edges[j] = eId;
            }
        }
    }
}

ISNP_API void compute_patches(const stl_vector_mp<stl_vector_mp<uint32_t>>& edges_of_face,
                              const stl_vector_mp<iso_edge_t>&              patch_edges,
                              const stl_vector_mp<polygon_face_t>&          patch_faces,
                              stl_vector_mp<stl_vector_mp<uint32_t>>&       patches,
                              stl_vector_mp<uint32_t>&                      patch_of_face_mapping)
{
    stl_vector_mp<bool> visited_face(edges_of_face.size(), false);
    for (uint32_t i = 0; i < edges_of_face.size(); i++) {
        if (!visited_face[i]) {
            // new patch
            auto&                patch    = patches.emplace_back();
            const auto           patch_id = static_cast<uint32_t>(patches.size() - 1);
            std::queue<uint32_t> Q{};
            Q.emplace(i);
            patch.emplace_back(i);
            visited_face[i]          = true;
            patch_of_face_mapping[i] = patch_id;
            while (!Q.empty()) {
                const auto fId = Q.front();
                Q.pop();
                for (const auto eId : edges_of_face[fId]) {
                    if (patch_edges[eId].headers.size() == 2) { // manifold edge
                        const auto other_fId = (patch_edges[eId].headers[0].face_index == fId)
                                                   ? patch_edges[eId].headers[1].face_index
                                                   : patch_edges[eId].headers[0].face_index;
                        if (!visited_face[other_fId]) {
                            Q.emplace(other_fId);
                            patch.emplace_back(other_fId);
                            patch_of_face_mapping[other_fId] = patch_id;
                            visited_face[other_fId]          = true;
                        }
                    }
                }
            }
        }
    }
}

ISNP_API void compute_chains(const stl_vector_mp<iso_edge_t>&              patch_edges,
                             const stl_vector_mp<stl_vector_mp<uint32_t>>& non_manifold_edges_of_vert,
                             stl_vector_mp<stl_vector_mp<uint32_t>>&       chains)
{
    stl_vector_mp<bool> visited_edge(patch_edges.size(), false);
    for (uint32_t i = 0; i < patch_edges.size(); i++) {
        if (!visited_edge[i] && patch_edges[i].headers.size() > 2) {
            // unvisited non-manifold iso-edge (not a boundary edge)
            // new chain
            auto&                chain = chains.emplace_back();
            std::queue<uint32_t> Q{};
            Q.emplace(i);
            chain.emplace_back(i);
            visited_edge[i] = true;
            while (!Q.empty()) {
                const auto eId = Q.front();
                Q.pop();
                // v1
                auto v = patch_edges[eId].v1;
                if (non_manifold_edges_of_vert[v].size() == 2) {
                    const auto other_eId = (non_manifold_edges_of_vert[v][0] == eId) ? non_manifold_edges_of_vert[v][1]
                                                                                     : non_manifold_edges_of_vert[v][0];
                    if (!visited_edge[other_eId]) {
                        Q.emplace(other_eId);
                        chain.emplace_back(other_eId);
                        visited_edge[other_eId] = true;
                    }
                }
                // v2
                v = patch_edges[eId].v2;
                if (non_manifold_edges_of_vert[v].size() == 2) {
                    const auto other_eId = (non_manifold_edges_of_vert[v][0] == eId) ? non_manifold_edges_of_vert[v][1]
                                                                                     : non_manifold_edges_of_vert[v][0];
                    if (!visited_edge[other_eId]) {
                        Q.emplace(other_eId);
                        chain.emplace_back(other_eId);
                        visited_edge[other_eId] = true;
                    }
                }
            }
        }
    }
}

ISNP_API void compute_shells_and_components(const stl_vector_mp<stl_vector_mp<uint32_t>>& half_patch_adj_list,
                                            stl_vector_mp<stl_vector_mp<uint32_t>>&       shells,
                                            stl_vector_mp<uint32_t>&                      shell_of_half_patch,
                                            stl_vector_mp<stl_vector_mp<uint32_t>>&       components,
                                            stl_vector_mp<uint32_t>&                      component_of_patch)
{
    const auto          num_patch = half_patch_adj_list.size() / 2;
    // find connected component of half-patch adjacency graph
    // each component is a shell
    stl_vector_mp<bool> visited_flags(2 * num_patch, false);
    shells.clear();
    shell_of_half_patch.resize(2 * num_patch);
    for (uint32_t i = 0; i < 2 * num_patch; i++) {
        if (!visited_flags[i]) {
            // create new component
            const uint32_t       shell_Id = static_cast<uint32_t>(shells.size());
            auto&                shell    = shells.emplace_back();
            std::queue<uint32_t> Q{};
            Q.emplace(i);
            shell.emplace_back(i);
            shell_of_half_patch[i] = shell_Id;
            visited_flags[i]       = true;
            while (!Q.empty()) {
                auto half_patch = Q.front();
                Q.pop();
                for (auto hp : half_patch_adj_list[half_patch]) {
                    if (!visited_flags[hp]) {
                        shell.emplace_back(hp);
                        shell_of_half_patch[hp] = shell_Id;
                        Q.emplace(hp);
                        visited_flags[hp] = true;
                    }
                }
            }
        }
    }
    // find connected component of patch-adjacency graph
    // each component is an iso-surface component
    visited_flags.clear();
    visited_flags.resize(num_patch, false);
    components.clear();
    component_of_patch.resize(num_patch);
    for (uint32_t i = 0; i < num_patch; i++) {
        if (!visited_flags[i]) {
            // create new component
            const uint32_t       component_Id = static_cast<uint32_t>(components.size());
            auto&                component    = components.emplace_back();
            std::queue<uint32_t> Q{};
            Q.emplace(i);
            component.emplace_back(i);
            component_of_patch[i] = component_Id;
            visited_flags[i]      = true;
            while (!Q.empty()) {
                auto patch = Q.front();
                Q.pop();
                // 1-side of patch
                for (auto hp : half_patch_adj_list[2 * patch]) {
                    if (!visited_flags[hp / 2]) {
                        const auto p = hp / 2;
                        component.emplace_back(p);
                        component_of_patch[p] = component_Id;
                        Q.emplace(p);
                        visited_flags[p] = true;
                    }
                }
                // -1-side of patch
                for (auto hp : half_patch_adj_list[2 * patch + 1]) {
                    if (!visited_flags[hp / 2]) {
                        const auto p = hp / 2;
                        component.emplace_back(p);
                        component_of_patch[p] = component_Id;
                        Q.emplace(p);
                        visited_flags[p] = true;
                    }
                }
            }
        }
    }
    // get shells as list of patch indices
    //    for (auto& shell : shells) {
    //        for (auto& pId : shell) {
    //            pId /= 2; // get patch index of half-patch
    //        }
    //    }
}

ISNP_API void compute_arrangement_cells(uint32_t                                            num_shell,
                                        const stl_vector_mp<std::pair<uint32_t, uint32_t>>& shell_links,
                                        stl_vector_mp<stl_vector_mp<uint32_t>>&             arrangement_cells)
{
    // build shell adjacency list
    uint32_t                                          sink_shell = num_shell;
    flat_hash_map_mp<uint32_t, std::vector<uint32_t>> adjacent_shells{};
    for (const auto& link : shell_links) {
        if (link.first == invalid_index) {
            adjacent_shells[sink_shell].emplace_back(link.second);
            adjacent_shells[link.second].emplace_back(sink_shell);
        } else if (link.second == invalid_index) {
            adjacent_shells[sink_shell].emplace_back(link.first);
            adjacent_shells[link.first].emplace_back(sink_shell);
        } else {
            adjacent_shells[link.first].emplace_back(link.second);
            adjacent_shells[link.second].emplace_back(link.first);
        }
    }

    // find connected components of shell adjacency graph
    // each component is an arrangement cells
    stl_vector_mp<bool> visited_shell(num_shell + 1, false);
    //    arrangement_cells.clear();
    for (uint32_t i = 0; i < num_shell + 1; ++i) {
        if (!visited_shell[i]) {
            // create new component
            auto&                arr_cell = arrangement_cells.emplace_back();
            std::queue<uint32_t> Q{};
            Q.emplace(i);
            arr_cell.emplace_back(i);
            visited_shell[i] = true;
            while (!Q.empty()) {
                const auto shell_id = Q.front();
                Q.pop();
                for (const auto s : adjacent_shells[shell_id]) {
                    if (!visited_shell[s]) {
                        arr_cell.emplace_back(s);
                        Q.emplace(s);
                        visited_shell[s] = true;
                    }
                }
            }
        }
    }

    // remove sink shell from arrangement cells
    stl_vector_mp<uint32_t> sink_free_shell_list{};
    for (auto& arr_cell : arrangement_cells) {
        sink_free_shell_list.clear();
        for (const auto s : arr_cell) {
            if (s < num_shell) { sink_free_shell_list.emplace_back(s); }
        }
        // arr_cell = sink_free_shell_list;
        std::swap(arr_cell, sink_free_shell_list);
    }
}