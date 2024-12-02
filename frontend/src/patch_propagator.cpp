#include <container/hashmap.hpp>

#include <internal_api.hpp>

#include "patch_propagator.hpp"

solve_result_t PatchPropagator::execute(const virtual_node_t&                         tree_root,
                                        const stl_vector_mp<uint32_t>&                leaf_index_of_primitive,
                                        const stl_vector_mp<raw_point_t>&             vertices,
                                        const stl_vector_mp<polygon_face_t>&          faces,
                                        const stl_vector_mp<stl_vector_mp<uint32_t>>& patches,
                                        const stl_vector_mp<double>&                  patch_areas,
                                        const stl_vector_mp<double>&                  volume_parts_of_patches,
                                        const stl_vector_mp<stl_vector_mp<uint32_t>>& arrangement_cells,
                                        const stl_vector_mp<uint32_t>&                shell_of_half_patch,
                                        const stl_vector_mp<stl_vector_mp<uint32_t>>& shells,
                                        stl_vector_mp<uint32_t>&                      output_polygon_faces,
                                        stl_vector_mp<uint32_t>&                      output_vertex_counts_of_face)
{
    solve_result_t result{};
    // for convience, now we do not shrink unused vertices
    // iff shrink, we'll need to remapping faces
    result.mesh.vertices     = reinterpret_cast<const raw_vector3d_t*>(vertices.data());
    result.mesh.num_vertices = static_cast<uint32_t>(vertices.size());

    const auto num_func = get_primitive_count();

    stl_vector_mp<uint32_t> shell_to_cell(shells.size());
    for (uint32_t i = 0; i < arrangement_cells.size(); i++) {
        for (auto shell : arrangement_cells[i]) shell_to_cell[shell] = i;
    }

    stl_vector_mp<dynamic_bitset_mp<>> function_cell_labels(num_func, dynamic_bitset_mp<>(arrangement_cells.size()));
    propagate_labels(vertices,
                     faces,
                     patches,
                     arrangement_cells,
                     shell_of_half_patch,
                     shells,
                     shell_to_cell,
                     function_cell_labels);

    auto active_cell_label = filter_cells_by_boolean(tree_root, leaf_index_of_primitive, function_cell_labels);

    stl_vector_mp<bool>  visited_cells(arrangement_cells.size(), false);
    std::queue<uint32_t> Q{};

    for (uint32_t i = 0; i < arrangement_cells.size(); ++i) {
        if (active_cell_label[i]) {
            Q.emplace(i);
            break;
        }
    }
    while (!Q.empty()) {
        const auto cell_index = Q.front();
        const auto cell       = arrangement_cells[cell_index];
        Q.pop();

        if (!visited_cells[cell_index] && active_cell_label[cell_index]) {
            visited_cells[cell_index] = true;
            for (auto shell : cell) {
                // it is secured that one shell cell cannot have a pair of half patches with same patch index
                for (auto half_patch : shells[shell]) {
                    bool sign        = (half_patch % 2 == 0) ? 0 : 1;
                    auto oppose_cell = shell_to_cell[shell_of_half_patch[!sign ? (half_patch + 1) : (half_patch - 1)]];

                    // iff not interior patch, we do surface propagation
                    if (!active_cell_label[oppose_cell]) {
                        result.mesh.num_faces += static_cast<uint32_t>(patches[half_patch / 2].size());
                        // NOTE: since patch inside the sdf should be oriented counterclockwise when viewed from inside
                        // i.e. it is viewed to be clockwise when viewed from outside
                        // so we need to flip its vertex order here
                        // NOTE: as for the integral, surface integral is always positive so we just add them up
                        // however for the partial volume integral, the right one should be corresponding to the normal which
                        // points outside. Here's a simple prove that the partial volume integral always have right sign: If the
                        // surface is outside (i.e. counterclockwise), then it is obviously right; If the surface is inside
                        // (i.e. clockwise), then its normal should point to the other side of that surface, which is the
                        // outside of that surface, so it is also right
                        if (!sign) {
                            for (const auto face : patches[half_patch / 2]) {
                                const auto& face_vertices = faces[face].vertex_indices;
                                output_vertex_counts_of_face.emplace_back(face_vertices.size());
                                output_polygon_faces.insert(output_polygon_faces.end(),
                                                            std::make_move_iterator(face_vertices.rbegin()),
                                                            std::make_move_iterator(face_vertices.rend()));
                            }
                        } else {
                            for (const auto face : patches[half_patch / 2]) {
                                const auto& face_vertices = faces[face].vertex_indices;
                                output_vertex_counts_of_face.emplace_back(face_vertices.size());
                                output_polygon_faces.insert(output_polygon_faces.end(),
                                                            std::make_move_iterator(face_vertices.begin()),
                                                            std::make_move_iterator(face_vertices.end()));
                            }
                        }
                        result.surf_int_result += patch_areas[half_patch / 2];
                    } else if (!visited_cells[oppose_cell]) {
                        // active and not visited cell, enqueue it
                        Q.emplace(oppose_cell);
                    }

                    // always accumulate the partial volume integral
                    result.vol_int_result += -volume_parts_of_patches[half_patch / 2];
                }
            }
        }
    }

    result.mesh.faces         = output_polygon_faces.data();
    result.mesh.vertex_counts = output_vertex_counts_of_face.data();
    return result;
}

void PatchPropagator::propagate_labels(const stl_vector_mp<raw_point_t>&             vertices,
                                       const stl_vector_mp<polygon_face_t>&          faces,
                                       const stl_vector_mp<stl_vector_mp<uint32_t>>& patches,
                                       const stl_vector_mp<stl_vector_mp<uint32_t>>& arrangement_cells,
                                       const stl_vector_mp<uint32_t>&                shell_of_half_patch,
                                       const stl_vector_mp<stl_vector_mp<uint32_t>>& shells,
                                       const stl_vector_mp<uint32_t>&                shell_to_cell,
                                       stl_vector_mp<dynamic_bitset_mp<>>&           function_cell_labels)
{
    const auto num_func = get_primitive_count();

    stl_vector_mp<bool>                                   visited_cells(arrangement_cells.size(), false);
    stl_vector_mp<bool>                                   visited_functions(num_func, false);
    stl_vector_mp<stl_vector_mp<uint32_t>>                inactive_func_stacks(num_func);
    std::queue<uint32_t>                                  Q{};
    flat_hash_map_mp<uint32_t, std::pair<uint32_t, bool>> cell_neighbors_map{};

    Q.emplace(0);
    while (!Q.empty()) {
        const auto cell_index = Q.front();
        const auto cell       = arrangement_cells[cell_index];
        Q.pop();

        if (!visited_cells[cell_index]) {
            visited_cells[cell_index] = true;
            cell_neighbors_map.clear();

            for (auto shell : cell) {
                for (auto half_patch : shells[shell]) {
                    auto function_label = faces[patches[half_patch / 2][0]].implicit_function_index;
                    // CAUTION: we assume that the sign is 1 when the surface is inside the sdf before
                    // but in blobtree we assume that the sign is 0 when the surface is inside the sdf
                    // so we need to flip the sign here
                    // bool sign           = (half_patch % 2 == 0) ? 1 : 0;
                    // auto oppose_cell    = shell_to_cell[shell_of_half_patch[sign ? (half_patch + 1) : (half_patch - 1)]];
                    bool sign           = (half_patch % 2 == 0) ? 0 : 1;
                    auto oppose_cell    = shell_to_cell[shell_of_half_patch[!sign ? (half_patch + 1) : (half_patch - 1)]];
                    cell_neighbors_map[oppose_cell] = std::make_pair(function_label, sign);

#ifdef RELEASE_BRANCH
                    if (visited_functions[function_label] != false
                        && function_cell_labels[function_label][cell_index] != sign) {
                        throw std::runtime_error("ERROR: Inconsistent Cell Function Labels.");
                    }
#endif

                    function_cell_labels[function_label][cell_index] = sign;
                    visited_functions[function_label]                = true;
                    // propagate the function signs to all previously inactive cells
                    if (inactive_func_stacks[function_label].size() > 0) {
                        for (const auto cell_iter : inactive_func_stacks[function_label]) {
                            function_cell_labels[function_label][cell_iter] = sign;
                        }
                        inactive_func_stacks[function_label].clear();
                    }
                }
            }

            // fetch inactive function index
            for (uint32_t func_iter = 0; func_iter < num_func; func_iter++) {
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

                const auto& [func_index, sign] = other_cell_func_label;
                for (uint32_t func_iter = 0; func_iter < func_index; ++func_iter)
                    function_cell_labels[func_iter][other_cell_index] = function_cell_labels[func_iter][cell_index];
                // opposite cell has opposite sign on same patch
                function_cell_labels[func_index][other_cell_index] = !sign;
                for (uint32_t func_iter = func_index + 1; func_iter < num_func; ++func_iter)
                    function_cell_labels[func_iter][other_cell_index] = function_cell_labels[func_iter][cell_index];
            }
        }
    }

    // for all unvisited functions, they must totally contain or be contained by some surfaces
    // so we can test their signs by testing whether the representative vertex is inside or outside the aabbs
    for (uint32_t i = 0; i < num_func; i++) {
        if (visited_functions[i] == false) {
            for (uint32_t j = 0; j < arrangement_cells.size(); ++j) {
                const auto& cell                  = arrangement_cells[j];
                const auto& representative_shell  = shells[cell[0]];
                const auto& representative_patch  = patches[representative_shell[0] / 2];
                const auto& representative_face   = faces[representative_patch[0]];
                const auto& representative_vertex = vertices[representative_face.vertex_indices[0]];

                function_cell_labels[i][j] = get_aabb(i).contains(representative_vertex);
            }
        }
    }
}

dynamic_bitset_mp<> PatchPropagator::filter_cells_by_boolean(const virtual_node_t&                     tree_root,
                                                             const stl_vector_mp<uint32_t>&            leaf_index_of_primitive,
                                                             const stl_vector_mp<dynamic_bitset_mp<>>& function_cell_labels)
{
    const auto                       num_funcs             = get_primitive_count();
    const auto                       num_cells             = function_cell_labels[0].size();
    const auto                       max_parent_node_index = tree_root.inner_index;
    std::vector<dynamic_bitset_mp<>> node_cell_labels(blobtree_get_node_count(tree_root.main_index),
                                                      dynamic_bitset_mp<>(num_cells));
    std::vector<bool>                node_visited(blobtree_get_node_count(tree_root.main_index), false);

    // move function cell labels to leaf node cell labels
    for (uint32_t func_iter = 0; func_iter < num_funcs; ++func_iter) {
        const auto& leaf_node_index       = leaf_index_of_primitive[func_iter];
        node_cell_labels[leaf_node_index] = std::move(function_cell_labels[func_iter]);
        node_visited[leaf_node_index]     = true;
    }

    // assume: input complete binary tree with root
    virtual_node_t current_node_index       = tree_root;
    uint32_t&      current_node_inner_index = current_node_index.inner_index;
    node_t         current_node;
    for (uint32_t func_iter = 0; func_iter < num_funcs; ++func_iter) {
        current_node_inner_index = leaf_index_of_primitive[func_iter];
        current_node             = blobtree_get_node(current_node_index);
        // find first unvisited parent node
        while (node_visited[current_node_inner_index]) {
            current_node_inner_index = node_fetch_parent_index(current_node);
            current_node             = blobtree_get_node(current_node_index);
        }

        while (!node_visited[current_node_inner_index]) {
            const uint32_t left_child_index  = node_fetch_left_child_index(current_node);
            const uint32_t right_child_index = node_fetch_right_child_index(current_node);

            // only process when both children has been processed
            if (!node_visited[left_child_index] || !node_visited[right_child_index]) break;

            node_visited[current_node_inner_index] = true;
            switch (node_fetch_operation(current_node)) {
                case eNodeOperation::unionOp:
                    node_cell_labels[current_node_inner_index] =
                        node_cell_labels[left_child_index] | node_cell_labels[right_child_index];
                    break;
                case eNodeOperation::intersectionOp:
                    node_cell_labels[current_node_inner_index] =
                        node_cell_labels[left_child_index] & node_cell_labels[right_child_index];
                    break;
                case eNodeOperation::differenceOp:
                    node_cell_labels[current_node_inner_index] =
                        node_cell_labels[left_child_index] & node_cell_labels[right_child_index].flip();
                    break;
                default: throw std::runtime_error("ERROR: Node operation set to unknown."); break;
            }

            if (current_node_inner_index == tree_root.inner_index) return node_cell_labels[current_node_inner_index];

            current_node_inner_index = node_fetch_parent_index(current_node);
            current_node             = blobtree_get_node(current_node_index);
        }
    }

    throw std::runtime_error("ERROR: Filter cells cannot reach root of input blobtree.");
    return dynamic_bitset_mp<>{};
}