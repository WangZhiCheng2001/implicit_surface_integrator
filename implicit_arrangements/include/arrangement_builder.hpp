#pragma once

#include "add_plane.hpp"
#include "implicit_arrangement.hpp"
#include "plane.hpp"
#include "ia_structure.hpp"
#include "lut.hpp"
#include "union_find_disjoint_set.hpp"
#include "extract_arrangement.hpp"

inline ia_complex_t init_ia_complex(uint32_t num_planes)
{
    ia_complex_t ia_complex{};
    ia_complex.vertices.resize(3 + 1);

    ia_complex.vertices[0] = {1, 2, 3};
    ia_complex.vertices[1] = {2, 3, 0};
    ia_complex.vertices[2] = {3, 0, 1};
    ia_complex.vertices[3] = {0, 1, 2};

    ia_complex.edges.resize(6);
    ia_complex.edges[0].vertices          = {0, 1};
    ia_complex.edges[1].vertices          = {0, 2};
    ia_complex.edges[2].vertices          = {0, 3};
    ia_complex.edges[3].vertices          = {1, 2};
    ia_complex.edges[4].vertices          = {1, 3};
    ia_complex.edges[5].vertices          = {2, 3};
    ia_complex.edges[0].supporting_planes = {2, 3};
    ia_complex.edges[1].supporting_planes = {1, 3};
    ia_complex.edges[2].supporting_planes = {1, 2};
    ia_complex.edges[3].supporting_planes = {0, 3};
    ia_complex.edges[4].supporting_planes = {0, 2};
    ia_complex.edges[5].supporting_planes = {0, 1};

    ia_complex.faces.resize(4);
    ia_complex.faces[0].edges            = {5, 3, 4};
    ia_complex.faces[1].edges            = {2, 1, 5};
    ia_complex.faces[2].edges            = {4, 0, 2};
    ia_complex.faces[3].edges            = {1, 0, 3};
    ia_complex.faces[0].supporting_plane = 0;
    ia_complex.faces[1].supporting_plane = 1;
    ia_complex.faces[2].supporting_plane = 2;
    ia_complex.faces[3].supporting_plane = 3;
    ia_complex.faces[0].positive_cell    = 0;
    ia_complex.faces[0].negative_cell    = INVALID_INDEX;
    ia_complex.faces[1].positive_cell    = 0;
    ia_complex.faces[1].negative_cell    = INVALID_INDEX;
    ia_complex.faces[2].positive_cell    = 0;
    ia_complex.faces[2].negative_cell    = INVALID_INDEX;
    ia_complex.faces[3].positive_cell    = 0;
    ia_complex.faces[3].negative_cell    = INVALID_INDEX;

    ia_complex.cells.resize(1);
    ia_complex.cells[0].faces = {0, 1, 2, 3};
    ia_complex.cells[0].signs = {true, true, true, true};
    ia_complex.cells[0].signs.resize(num_planes, false);

    return ia_complex;
}

class arrangement_builder
{
public:
    arrangement_builder(const tbb_vector_mp<plane_t>& planes)
    {
        const auto  num_planes = static_cast<uint32_t>(planes.size());
        const auto* ia         = lookup(planes);
        if (ia != nullptr) {
            m_arrangement = *ia;
        } else {
            auto ia_complex = init_ia_complex(num_planes + 3 + 1);
            m_planes        = plane_group_t(planes);
            m_coplanar_planes.init(num_planes + 3 + 1);
            uint32_t unique_plane_count = 0;
            for (size_t i = 0; i < num_planes; i++) {
                auto plane_id          = static_cast<uint32_t>(i + 3 + 1);
                auto coplanar_plane_id = add_plane(m_planes, ia_complex, plane_id);
                if (coplanar_plane_id != INVALID_INDEX) {
                    m_coplanar_planes.merge(plane_id, coplanar_plane_id);
                } else {
                    unique_plane_count++;
                }
            }
            
            m_arrangement = extract_arrangement(std::move(ia_complex));

            if (unique_plane_count != num_planes) {
                // Only popularize unqiue plane structure if duplicate planes are
                // detected.
                extract_unique_planes();
            }
        }
    }

    const arrangement_t& get_arrangement() const noexcept { return m_arrangement; }

    arrangement_t& get_arrangement() noexcept { return m_arrangement; }

    auto&& export_arrangement() && noexcept { return std::move(m_arrangement); }

private:
    const arrangement_t* lookup(const tbb_vector_mp<plane_t>& planes) const
    {
        extern stl_vector_mp<arrangement_t> ia_data;
        extern stl_vector_mp<uint32_t>      ia_indices;

        const auto num_planes = static_cast<uint32_t>(planes.size());
        if (num_planes == 1) {
            const auto outer_index = ia_compute_outer_index(planes[0]);
            if (outer_index == INVALID_INDEX) return nullptr;

            const auto start_idx = ia_indices[outer_index];
            assert(ia_indices[outer_index + 1] == start_idx + 1);

            return &ia_data[start_idx];
        } else if (num_planes == 2) {
            const auto outer_index = ia_compute_outer_index(planes[0], planes[1]);
            if (outer_index == INVALID_INDEX) return nullptr;

            const auto start_idx = ia_indices[outer_index];
            const auto end_idx   = ia_indices[outer_index + 1];

            if (end_idx == start_idx + 1) {
                return &ia_data[start_idx];
            } else if (end_idx > start_idx) {
                const auto inner_index = ia_compute_inner_index(outer_index, planes[0], planes[1]);
                if (inner_index == INVALID_INDEX) return nullptr;
                assert(inner_index < end_idx - start_idx);
                return &ia_data[start_idx + inner_index];
            }
        }

        return nullptr;
    }

    void extract_unique_planes()
    {
        auto is_plane_consistently_oriented = [&](uint32_t i1, uint32_t i2) -> bool {
            const auto& p1 = m_planes.get_plane(i1);
            const auto& p2 = m_planes.get_plane(i2);

            for (size_t i = 0; i < 3 + 1; i++) {
                const auto v1 = p1[i];
                const auto v2 = p2[i];
                if (v1 == 0 && v2 == 0) continue;
                return (v1 > 0 && v2 > 0) || (v1 < 0 && v2 < 0);
            }
            // plane p1 and p2 are consistently 0 over the cell.
            return true;
        };

        auto& r = m_arrangement;
        m_coplanar_planes.extract_disjoint_sets(r.unique_planes, r.unique_plane_indices);
        r.unique_plane_orientations.resize(r.unique_plane_indices.size(), true);
        for (const auto& planes : r.unique_planes) {
            const auto num_planes = planes.size();
            assert(num_planes > 0);
            r.unique_plane_orientations[planes[0]] = true;
            for (size_t i = 1; i < num_planes; ++i)
                r.unique_plane_orientations[planes[i]] = is_plane_consistently_oriented(planes[0], planes[i]);
        }
    }

    plane_group_t        m_planes{};
    UnionFindDisjointSet m_coplanar_planes{};
    arrangement_t        m_arrangement{};
};