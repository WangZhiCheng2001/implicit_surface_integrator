#include <algorithm/glue_algorithm.hpp>

#include "ia_cut_face.hpp"
#include "ia_structure.hpp"
#include "implicit_arrangement.hpp"
#include "robust_assert.hpp"

template <typename T>
inline void shrink(stl_vector_mp<T>& c, stl_vector_mp<uint32_t>& index_map, const stl_vector_mp<bool>& active_flags)
{
    const size_t s = c.size();
    index_map.assign(s, INVALID_INDEX);
    uint32_t active_count = 0;
    for (size_t i = 0; i < s; i++) {
        if (!active_flags[i]) continue;

        if (i != active_count) { std::swap(c[active_count], c[i]); }
        index_map[i] = active_count;
        active_count++;
    }
    c.resize(active_count);
}

/**
 * Remove unused verices and faces.
 */
inline void remove_unused_geometry(ia_complex_t& data)
{
    stl_vector_mp<bool>     active_geometries{};
    stl_vector_mp<uint32_t> index_map{};

    {
        // Shrink faces.
        active_geometries.assign(data.faces.size(), false);
        for (auto& c : data.cells) {
            for (auto fid : c.faces) { active_geometries[fid] = true; }
        }

        shrink(data.faces, index_map, active_geometries);

        for (auto& c : data.cells) {
            algorithm::transform<algorithm::ExecutionPolicySelector::simd_only>(
                c.faces.begin(),
                c.faces.end(),
                c.faces.begin(),
                [&](uint32_t i) {
                    ROBUST_ASSERT(index_map[i] != INVALID_INDEX);
                    return index_map[i];
                });
        }
    }

    // Shrink edges.
    {
        active_geometries.assign(data.edges.size(), false);
        for (auto& f : data.faces) {
            for (auto eid : f.edges) { active_geometries[eid] = true; }
        }

        shrink(data.edges, index_map, active_geometries);

        for (auto& f : data.faces) {
            algorithm::transform<algorithm::ExecutionPolicySelector::simd_only>(
                f.edges.begin(),
                f.edges.end(),
                f.edges.begin(),
                [&](uint32_t i) {
                    ROBUST_ASSERT(index_map[i] != INVALID_INDEX);
                    return index_map[i];
                });
        }
    }

    // Shrink vertices.
    {
        active_geometries.assign(data.vertices.size(), false);
        for (auto& e : data.edges) {
            for (auto vid : e.vertices) {
                ROBUST_ASSERT(vid != INVALID_INDEX);
                active_geometries[vid] = true;
            }
        }

        shrink(data.vertices, index_map, active_geometries);

        for (auto& e : data.edges) {
            e.vertices[0] = index_map[e.vertices[0]];
            e.vertices[1] = index_map[e.vertices[1]];
        }
    }
}

uint32_t add_plane(const plane_group_t& repo, ia_complex_t& ia_complex, uint32_t plane_index)
{
    const size_t num_vertices = ia_complex.vertices.size();
    const size_t num_edges    = ia_complex.edges.size();
    const size_t num_faces    = ia_complex.faces.size();
    const size_t num_cells    = ia_complex.cells.size();

    auto& vertices = ia_complex.vertices;
    auto& edges    = ia_complex.edges;
    auto& faces    = ia_complex.faces;
    auto& cells    = ia_complex.cells;

    // Reserve capacity. TODO: check if this helps.
    vertices.reserve(num_vertices + num_edges);
    edges.reserve(num_edges * 2);
    faces.reserve(num_faces * 2);
    cells.reserve(num_cells * 2);

    // Step 1: handle 0-faces.
    stl_vector_mp<int8_t> orientations{};
    orientations.reserve(num_vertices);
    for (uint32_t i = 0; i < num_vertices; i++) { orientations.emplace_back(ia_cut_0_face(repo, ia_complex, i, plane_index)); }

    // Step 2: handle 1-faces.
    stl_vector_mp<std::array<uint32_t, 3>> subedges{};
    subedges.reserve(num_edges);
    for (uint32_t i = 0; i < num_edges; i++) { subedges.emplace_back(ia_cut_1_face(ia_complex, i, plane_index, orientations)); }

    // Step 3: handle 2-faces.
    stl_vector_mp<std::array<uint32_t, 3>> subfaces{};
    subfaces.reserve(num_faces);
    for (uint32_t i = 0; i < num_faces; i++) {
        subfaces.emplace_back(ia_cut_2_face(ia_complex, i, plane_index, orientations, subedges));
    }

    // Step 4: handle 3-faces.
    stl_vector_mp<std::array<uint32_t, 3>> subcells{};
    subcells.reserve(num_cells);
    for (uint32_t i = 0; i < num_cells; i++) { subcells.emplace_back(ia_cut_3_face(ia_complex, i, plane_index, subfaces)); }

    // Step 5: remove old cells and update cell indices
    {
        stl_vector_mp<bool> to_keep(cells.size(), false);
        for (const auto& subcell : subcells) {
            if (subcell[0] != INVALID_INDEX) to_keep[subcell[0]] = true;
            if (subcell[1] != INVALID_INDEX) to_keep[subcell[1]] = true;
        }

        stl_vector_mp<uint32_t> index_map{};
        shrink(cells, index_map, to_keep);

        // Update cell indices in faces.
        for (auto& f : faces) {
            if (f.positive_cell != INVALID_INDEX) f.positive_cell = index_map[f.positive_cell];
            if (f.negative_cell != INVALID_INDEX) f.negative_cell = index_map[f.negative_cell];
        }
    }

    // Step 6: check for coplanar planes.
    uint32_t coplanar_plane = INVALID_INDEX;
    for (uint32_t i = 0; i < num_faces; i++) {
        const auto& subface = subfaces[i];
        if (subface[0] == INVALID_INDEX && subface[1] == INVALID_INDEX) {
            const auto& f  = faces[i];
            coplanar_plane = f.supporting_plane;
        }
    }

    // Step 7: remove unused geometries.
    remove_unused_geometry(ia_complex);

    return coplanar_plane;
}