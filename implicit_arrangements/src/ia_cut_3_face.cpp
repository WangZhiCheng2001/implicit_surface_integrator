#include <container/hashmap.hpp>
#include <algorithm/glue_algorithm.hpp>

#include "ia_cut_face.hpp"
#include "implicit_arrangement.hpp"
#include "robust_assert.hpp"

std::array<uint32_t, 3> ia_cut_3_face(ia_complex_t&                                 ia_complex,
                                      uint32_t                                      cid,
                                      uint32_t                                      plane_index,
                                      const stl_vector_mp<std::array<uint32_t, 3>>& subfaces)
{
    auto& edges = ia_complex.edges;
    auto& faces = ia_complex.faces;
    auto& cells = ia_complex.cells;

    const auto& cell               = cells[cid];
    const auto  num_boundary_faces = cell.faces.size();

    uint32_t                cut_face_id = INVALID_INDEX;
    stl_vector_mp<uint32_t> positive_subfaces{};
    stl_vector_mp<uint32_t> negative_subfaces{};
    stl_vector_mp<uint32_t> cut_edges{};
    stl_vector_mp<bool>     cut_edge_orientations{};
    positive_subfaces.reserve(num_boundary_faces + 1);
    negative_subfaces.reserve(num_boundary_faces + 1);
    cut_edges.reserve(num_boundary_faces);
    cut_edge_orientations.reserve(num_boundary_faces);

    auto compute_cut_edge_orientation = [&](uint32_t fid, const std::array<uint32_t, 3>& subface) -> bool {
        ROBUST_ASSERT(subface[2] != INVALID_INDEX);
        const auto& f = faces[fid];
        bool        s = cell.signs[f.supporting_plane];

        if (subface[0] == INVALID_INDEX || subface[1] == INVALID_INDEX) {
            // Intersection edge is on the boundary of the face.
            auto itr =
                algorithm::find<algorithm::ExecutionPolicySelector::simd_only>(f.edges.begin(), f.edges.end(), subface[2]);
            ROBUST_ASSERT(itr != f.edges.end());
            size_t curr_i = itr - f.edges.begin();
            size_t next_i = (curr_i + 1) % f.edges.size();

            const auto& curr_e = edges[f.edges[curr_i]];
            const auto& next_e = edges[f.edges[next_i]];
            bool        edge_is_consistent_with_face =
                (curr_e.vertices[1] == next_e.vertices[0] || curr_e.vertices[1] == next_e.vertices[1]);

            bool on_positive_side = subface[0] != INVALID_INDEX;

            uint8_t key = static_cast<uint8_t>(s) + static_cast<uint8_t>(edge_is_consistent_with_face)
                          + static_cast<uint8_t>(on_positive_side);
            return key % 2 == 0;
        } else {
            // Intersection edge is a cross cut.
            return !s;
        }
    };

    for (auto fid : cell.faces) {
        const auto& subface = subfaces[fid];
        if (subface[0] == INVALID_INDEX && subface[1] == INVALID_INDEX) { cut_face_id = fid; }
        if (subface[0] != INVALID_INDEX) { positive_subfaces.emplace_back(subface[0]); }
        if (subface[1] != INVALID_INDEX) { negative_subfaces.emplace_back(subface[1]); }
        if (subface[2] != INVALID_INDEX) {
            cut_edges.emplace_back(subface[2]);
            cut_edge_orientations.emplace_back(compute_cut_edge_orientation(fid, subface));
        }
    }

    if (positive_subfaces.empty() && negative_subfaces.empty()) {
        // The implicit function is identical over the whole cell.
        return {INVALID_INDEX, INVALID_INDEX, INVALID_INDEX};
    } else if (positive_subfaces.empty()) {
        cells[cid].signs[plane_index] = false;
        return {INVALID_INDEX, cid, cut_face_id};
    } else if (negative_subfaces.empty()) {
        cells[cid].signs[plane_index] = true;
        return {cid, INVALID_INDEX, cut_face_id};
    }

    // Chain cut edges into a loop.
    {
        size_t num_cut_edges = cut_edges.size();
        ROBUST_ASSERT(num_cut_edges >= 3);
        flat_hash_map_mp<uint32_t, uint32_t> v2e{};
        v2e.reserve(num_cut_edges);
        for (size_t i = 0; i < num_cut_edges; i++) {
            const auto  eid = cut_edges[i];
            const auto& e   = edges[eid];
            if (cut_edge_orientations[i]) {
                v2e[e.vertices[0]] = i;
            } else {
                v2e[e.vertices[1]] = i;
            }
        }
        stl_vector_mp<uint32_t> chained_cut_edges{};
        chained_cut_edges.reserve(num_cut_edges);
        chained_cut_edges.emplace_back(0u);
        while (chained_cut_edges.size() < num_cut_edges) {
            const uint32_t i   = chained_cut_edges.back();
            const auto&    e   = edges[cut_edges[i]];
            const auto     vid = cut_edge_orientations[i] ? e.vertices[1] : e.vertices[0];
            const auto     itr = v2e.find(vid);
            ROBUST_ASSERT(itr != v2e.end());
            const auto next_i = itr->second;
            if (cut_edges[next_i] == cut_edges[chained_cut_edges.front()]) { break; }
            chained_cut_edges.emplace_back(next_i);
        }
        algorithm::transform<algorithm::ExecutionPolicySelector::simd_only>(chained_cut_edges.begin(),
                                                                            chained_cut_edges.end(),
                                                                            chained_cut_edges.begin(),
                                                                            [&](auto i) { return cut_edges[i]; });
        std::swap(cut_edges, chained_cut_edges);
    }

    // Cross cut.
    ROBUST_ASSERT(!cut_edges.empty());
    ia_face_t cut_face;
    cut_face.edges            = std::move(cut_edges);
    cut_face.supporting_plane = plane_index;
    faces.emplace_back(std::move(cut_face));
    cut_face_id = faces.size() - 1;

    // Generate positive and negative subcell.
    ia_cell_t positive_cell, negative_cell;
    positive_cell.faces.reserve(positive_subfaces.size() + 1);
    negative_cell.faces.reserve(negative_subfaces.size() + 1);

    positive_subfaces.emplace_back(cut_face_id);
    positive_cell.faces              = std::move(positive_subfaces);
    positive_cell.signs              = cell.signs;
    positive_cell.signs[plane_index] = true;

    negative_subfaces.emplace_back(cut_face_id);
    negative_cell.faces              = std::move(negative_subfaces);
    negative_cell.signs              = cell.signs;
    negative_cell.signs[plane_index] = false;

    cells.emplace_back(std::move(positive_cell));
    cells.emplace_back(std::move(negative_cell));
    uint32_t positive_cell_id = static_cast<uint32_t>(cells.size() - 2);
    uint32_t negative_cell_id = static_cast<uint32_t>(cells.size() - 1);

    // Update cell id on each side of involved faces.
    {
        // cut face
        ROBUST_ASSERT(cut_face_id != INVALID_INDEX);
        auto& cut_f         = faces[cut_face_id];
        cut_f.positive_cell = positive_cell_id;
        cut_f.negative_cell = negative_cell_id;

        auto& positive_c = cells[positive_cell_id];
        auto& negative_c = cells[negative_cell_id];

        for (auto fid : positive_c.faces) {
            if (fid == cut_face_id) continue;
            auto& f = faces[fid];
            ROBUST_ASSERT(f.positive_cell == cid || f.negative_cell == cid);
            if (f.positive_cell == cid) {
                f.positive_cell = positive_cell_id;
            } else {
                f.negative_cell = positive_cell_id;
            }
        }
        for (auto fid : negative_c.faces) {
            if (fid == cut_face_id) continue;
            auto& f = faces[fid];
            ROBUST_ASSERT(f.positive_cell == cid || f.negative_cell == cid);
            if (f.positive_cell == cid) {
                f.positive_cell = negative_cell_id;
            } else {
                f.negative_cell = negative_cell_id;
            }
        }
    }

    return {positive_cell_id, negative_cell_id, cut_face_id};
}