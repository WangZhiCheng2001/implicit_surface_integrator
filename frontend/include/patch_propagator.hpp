#pragma once

#include <container/dynamic_bitset.hpp>

#include <utils/fwd_types.hpp>

#include "execution.h"

/// Propagate the function labels of patches to cells.
class PatchPropagator
{
public:
    solve_result_t execute(const virtual_node_t&                         tree_root,
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
                           stl_vector_mp<uint32_t>&                      output_vertex_counts_of_face);

private:
    void propagate_labels(const stl_vector_mp<raw_point_t>&             vertices,
                          const stl_vector_mp<polygon_face_t>&          faces,
                          const stl_vector_mp<stl_vector_mp<uint32_t>>& patches,
                          const stl_vector_mp<stl_vector_mp<uint32_t>>& arrangement_cells,
                          const stl_vector_mp<uint32_t>&                shell_of_half_patch,
                          const stl_vector_mp<stl_vector_mp<uint32_t>>& shells,
                          const stl_vector_mp<uint32_t>&                shell_to_cell,
                          stl_vector_mp<dynamic_bitset_mp<>>&           function_cell_labels);

    dynamic_bitset_mp<> filter_cells_by_boolean(const virtual_node_t&                     tree_root,
                                                const stl_vector_mp<uint32_t>&            leaf_index_of_primitive,
                                                const stl_vector_mp<dynamic_bitset_mp<>>& function_cell_labels);
};