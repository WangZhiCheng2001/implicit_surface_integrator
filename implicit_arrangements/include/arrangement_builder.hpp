#pragma once

#include "add_plane.hpp"
#include "common_structure.hpp"
#include "plane.hpp"
#include "ia_structure.hpp"
#include "lut.hpp"
#include "union_find_disjoint_set.hpp"
#include "extract_arrangement.hpp"

namespace detail
{
template <size_t N>
struct deduce_arrangement_type;

template <>
struct deduce_arrangement_type<2> {
    using type = Arrangement2D;
};

template <>
struct deduce_arrangement_type<3> {
    using type = Arrangement3D;
};

template <size_t N>
struct deduce_arrangement_result_type;

template <>
struct deduce_arrangement_result_type<2> {
    using type = Arrangement2DResult;
};

template <>
struct deduce_arrangement_result_type<3> {
    using type = Arrangement3DResult;
};
} // namespace detail

template <size_t N>
IAComplex<N> init_ia_complex(uint32_t num_planes)
{
    IAComplex<N> ia_complex;
    ia_complex.vertices.resize(N + 1);

    if constexpr (N == 2) {
        ia_complex.vertices[0] = {1, 2};
        ia_complex.vertices[1] = {2, 0};
        ia_complex.vertices[2] = {0, 1};

        ia_complex.edges.resize(3);
        ia_complex.edges[0].vertices         = {2, 1};
        ia_complex.edges[1].vertices         = {0, 2};
        ia_complex.edges[2].vertices         = {1, 0};
        ia_complex.edges[0].supporting_plane = 0;
        ia_complex.edges[1].supporting_plane = 1;
        ia_complex.edges[2].supporting_plane = 2;
        ia_complex.edges[0].positive_face    = 0;
        ia_complex.edges[0].negative_face    = INVALID_INDEX;
        ia_complex.edges[1].positive_face    = 0;
        ia_complex.edges[1].negative_face    = INVALID_INDEX;
        ia_complex.edges[2].positive_face    = 0;
        ia_complex.edges[2].negative_face    = INVALID_INDEX;

        ia_complex.faces.resize(1);
        ia_complex.faces[0].edges = {0, 1, 2};
        ia_complex.faces[0].signs = {true, true, true};
        ia_complex.faces[0].signs.resize(num_planes, false);
    } else {
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
    }
    return ia_complex;
}

template <size_t N>
class ArrangementBuilder
{
public:
    using plane_type              = typename detail::deduce_plane_type<N>::type;
    using arrangement_type        = typename detail::deduce_arrangement_type<N>::type;
    using arrangement_result_type = typename detail::deduce_arrangement_result_type<N>::type;

    ArrangementBuilder(const plane_type* planes, const uint32_t num_planes)
        : m_planes(planes, planes + num_planes), m_coplanar_planes(num_planes + N + 1)
    {
        const auto* ia = lookup(planes, num_planes);
        if (ia != nullptr) {
            m_arrangement.arrangement         = *ia;
            m_arrangement.is_runtime_computed = false;
        } else {
            auto     ia_complex         = init_ia_complex<N>(num_planes);
            uint32_t unique_plane_count = 0;
            for (size_t i = 0; i < num_planes; i++) {
                auto plane_id          = static_cast<uint32_t>(i + N + 1);
                auto coplanar_plane_id = add_plane(m_planes, ia_complex, plane_id);
                if (coplanar_plane_id != INVALID_INDEX) {
                    m_coplanar_planes.merge(plane_id, coplanar_plane_id);
                } else {
                    unique_plane_count++;
                }
            }

            m_arrangement.arrangement         = extract_arrangement(std::move(ia_complex));
            m_arrangement.is_runtime_computed = true;

            if (unique_plane_count != num_planes) {
                // Only popularize unqiue plane structure if duplicate planes are
                // detected.
                extract_unique_planes();
            }
        }
    }

    const arrangement_type& get_arrangement() const noexcept { return m_arrangement.arrangement; }

    arrangement_type& get_arrangement() noexcept { return m_arrangement.arrangement; }

    auto&& export_arrangement() && noexcept { return std::move(m_arrangement); }

    bool is_arrangement_precomputed() const noexcept { return m_arrangement.is_runtime_computed; }

private:
    const arrangement_type* lookup(const plane_type* planes, const uint32_t num_planes) const
    {
        extern std::vector<Arrangement3D> ia_data;
        extern std::vector<uint32_t>      ia_indices;

        if constexpr (N == 3) {
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
        }
        return nullptr;
    }

    void extract_unique_planes()
    {
        auto is_plane_consistently_oriented = [&](uint32_t i1, uint32_t i2) -> bool {
            const auto& p1 = m_planes.get_plane(i1);
            const auto& p2 = m_planes.get_plane(i2);

            for (size_t i = 0; i < N + 1; i++) {
                const auto v1 = (&p1.f0)[i];
                const auto v2 = (&p2.f0)[i];
                if (v1 == 0 && v2 == 0) continue;
                return (v1 > 0 && v2 > 0) || (v1 < 0 && v2 < 0);
            }
            // plane p1 and p2 are consistently 0 over the cell.
            return true;
        };

        auto& r = m_arrangement.arrangement;
        m_coplanar_planes.extract_disjoint_sets(r.unique_planes,
                                                r.unique_plane_indices,
                                                r.unique_plane_count,
                                                r.num_unique_planes);
        r.unique_plane_orientations =
            static_cast<bool*>(ScalableMemoryPoolSingleton::instance().malloc(sizeof(bool) * r.num_unique_planes));
        for (size_t i = 0; i < r.num_unique_planes; ++i) {
            const auto num_planes = r.unique_plane_count[i];
            assert(num_planes > 0);
            for (size_t j = 1; j < num_planes; ++j)
                r.unique_plane_orientations[r.unique_planes[i][j]] =
                    is_plane_consistently_oriented(r.unique_planes[i][0], r.unique_planes[i][j]);
        }
    }

    PlaneGroup<N>           m_planes{};
    UnionFindDisjointSet    m_coplanar_planes{};
    arrangement_result_type m_arrangement{};
    bool                    m_is_arrangement_precomputed{true};
};