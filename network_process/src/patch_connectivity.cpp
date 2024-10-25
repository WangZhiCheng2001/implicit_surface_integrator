#include <queue>

#include <container/hashmap.hpp>

#include <patch_connectivity.hpp>
#include "container/dynamic_bitset.hpp"
#include "utils/fwd_types.hpp"

void compute_patch_edges(const stl_vector_mp<PolygonFace>&         patch_faces,
                         stl_vector_mp<small_vector_mp<uint32_t>>& edges_of_face,
                         stl_vector_mp<IsoEdge>&                   patch_edges)
{
    uint32_t max_num_edge = 0;
    for (const auto& iso_face : patch_faces) { max_num_edge += iso_face.vertex_indices.size(); }
    patch_edges.reserve(max_num_edge / 2);
    edges_of_face.reserve(patch_faces.size());
    uint32_t                                                  num_iso_edge{};
    // map: (v1, v2) -> iso-edge index
    flat_hash_map_mp<std::pair<uint32_t, uint32_t>, uint32_t> edge_id{};
    for (uint32_t i = 0; i < patch_faces.size(); i++) {
        auto&          face       = patch_faces[i];
        const uint32_t num_edge   = face.vertex_indices.size();
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

void compute_patches(const stl_vector_mp<small_vector_mp<uint32_t>>& edges_of_face,
                     const stl_vector_mp<IsoEdge>&                   patch_edges,
                     const stl_vector_mp<PolygonFace>&               patch_faces,
                     stl_vector_mp<small_vector_mp<uint32_t>>&       patches,
                     stl_vector_mp<uint32_t>&                        patch_function_label)
{
    dynamic_bitset_mp<> visited_face(edges_of_face.size(), false);
    for (uint32_t i = 0; i < edges_of_face.size(); i++) {
        if (!visited_face[i]) {
            // new patch
            auto&                patch = patches.emplace_back();
            std::queue<uint32_t> Q{};
            Q.emplace(i);
            patch.emplace_back(i);
            visited_face[i] = true;
            patch_function_label.emplace_back(patch_faces[Q.front()].implicit_function_index);
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
                            visited_face[other_fId] = true;
                        }
                    }
                }
            }
        }
    }
}

void compute_chains(const stl_vector_mp<IsoEdge>&                   patch_edges,
                    const stl_vector_mp<small_vector_mp<uint32_t>>& non_manifold_edges_of_vert,
                    stl_vector_mp<small_vector_mp<uint32_t>>&       chains)
{
    dynamic_bitset_mp<> visited_edge(patch_edges.size(), false);
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

void compute_shells_and_components(uint32_t                                                 num_patch,
                                   const stl_vector_mp<small_vector_mp<half_patch_pair_t>>& half_patch_pair_list,
                                   stl_vector_mp<stl_vector_mp<uint32_t>>&                  shells,
                                   stl_vector_mp<uint32_t>&                                 shell_of_half_patch,
                                   stl_vector_mp<small_vector_mp<uint32_t>>&                components,
                                   stl_vector_mp<uint32_t>&                                 component_of_patch)
{
    // (patch i, 1) <--> 2i,  (patch i, -1) <--> 2i+1
    // compute half-patch adjacency list
    stl_vector_mp<small_vector_mp<uint32_t>> half_patch_adj_list(2 * num_patch);
    for (const auto& half_patch_pairs : half_patch_pair_list) {
        for (uint32_t i = 0; i < half_patch_pairs.size(); i++) {
            const auto& [hp1, hp2] = half_patch_pairs[i];
            // half-patch index of hp1
            const auto hp_Id1      = (hp1.orientation == 1) ? 2 * hp1.index : (2 * hp1.index + 1);
            // half-patch index of hp2
            const auto hp_Id2      = (hp2.orientation == 1) ? 2 * hp2.index : (2 * hp2.index + 1);
            half_patch_adj_list[hp_Id1].emplace_back(hp_Id2);
            half_patch_adj_list[hp_Id2].emplace_back(hp_Id1);
        }
    }
    // find connected component of half-patch adjacency graph
    // each component is a shell
    dynamic_bitset_mp<> visited_flags(2 * num_patch, false);
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

void compute_arrangement_cells(uint32_t                                            num_shell,
                               const stl_vector_mp<std::pair<uint32_t, uint32_t>>& shell_links,
                               stl_vector_mp<small_vector_mp<uint32_t>>&           arrangement_cells)
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
    small_dynamic_bitset_mp<> visited_shell(num_shell + 1, false);
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
    small_vector_mp<uint32_t> sink_free_shell_list{};
    for (auto& arr_cell : arrangement_cells) {
        sink_free_shell_list.clear();
        for (const auto s : arr_cell) {
            if (s < num_shell) { sink_free_shell_list.emplace_back(s); }
        }
        arr_cell = sink_free_shell_list;
    }
}

stl_vector_mp<small_dynamic_bitset_mp<>> sign_propagation(const stl_vector_mp<small_vector_mp<uint32_t>>& arrangement_cells,
                                                          const stl_vector_mp<uint32_t>&                  shell_of_half_patch,
                                                          const stl_vector_mp<small_vector_mp<uint32_t>>& shells,
                                                          const stl_vector_mp<uint32_t>&                  patch_function_label,
                                                          uint32_t                                        n_func,
                                                          const small_dynamic_bitset_mp<>&                sample_function_label)
{
    stl_vector_mp<uint32_t> shell_to_cell(shells.size());
    for (uint32_t i = 0; i < arrangement_cells.size(); i++) {
        for (auto shell : arrangement_cells[i]) { shell_to_cell[shell] = i; }
    }
    stl_vector_mp<small_dynamic_bitset_mp<>>              cell_function_label(arrangement_cells.size(),
                                                                 small_dynamic_bitset_mp<>(n_func, false));
    dynamic_bitset_mp<>                                   visited_cells(arrangement_cells.size(), false);
    small_dynamic_bitset_mp<>                             visited_functions(n_func, false);
    stl_vector_mp<small_vector_mp<uint32_t>>              inactive_func_stacks(n_func);
    std::queue<uint32_t>                                  Q{};
    flat_hash_map_mp<uint32_t, std::pair<uint32_t, bool>> cell_neighbors_map{};

    Q.emplace(0);
    while (!Q.empty()) {
        const auto cell_index = Q.front();
        const auto cell       = arrangement_cells[cell_index];
        Q.pop();
        if (!visited_cells[cell_index]) {
            visited_cells[cell_index] = true;
            auto& current_label       = cell_function_label[cell_index];
            cell_neighbors_map.clear();
            for (auto shell : cell) {
                for (auto half_patch : shells[shell]) {
                    auto function_label = patch_function_label[half_patch / 2];
                    bool sign           = (half_patch % 2 == 0) ? 1 : 0;
                    auto oppose_cell    = shell_to_cell[shell_of_half_patch[sign ? (half_patch + 1) : (half_patch - 1)]];
                    cell_neighbors_map[oppose_cell] = std::make_pair(function_label, sign);
                    if (visited_functions[function_label] != false && current_label[function_label] != sign) {
                        throw std::runtime_error("ERROR: Inconsistent Cell Function Labels.");
                    }
                    current_label[function_label]     = sign;
                    visited_functions[function_label] = true;
                    // propagate the function signs to all previously inactive cells
                    if (inactive_func_stacks[function_label].size() > 0) {
                        for (const auto cell_iter : inactive_func_stacks[function_label]) {
                            cell_function_label[cell_iter][function_label] = sign;
                        }
                        inactive_func_stacks[function_label].clear();
                    }
                }
            }
            // fetch inactive function index
            for (uint32_t func_iter = 0; func_iter < current_label.size(); func_iter++) {
                if (visited_functions[func_iter] == false) {
                    inactive_func_stacks[func_iter].emplace_back(cell_index);
                    for (const auto& [other_cell_index, _] : cell_neighbors_map) {
                        inactive_func_stacks[func_iter].emplace_back(other_cell_index);
                    }
                }
            }
            // propagate to neighboring cells
            for (const auto& [other_cell_index, other_cell_func_label] : cell_neighbors_map) {
                if (!visited_cells[other_cell_index]) Q.emplace(other_cell_index);
                cell_function_label[other_cell_index]                              = current_label;
                cell_function_label[other_cell_index][other_cell_func_label.first] = !other_cell_func_label.second;
            }
        }
    }

    for (uint32_t i = 0; i < n_func; i++) {
        if (visited_functions[i] == false) {
            for (auto& label : cell_function_label) { label[i] = sample_function_label[i]; }
        }
    }

    return cell_function_label;
}