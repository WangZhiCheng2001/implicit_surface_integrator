#include <bitset>

#include <implicit_predicates.h>

#include "lut.hpp"
#include "common_structure.hpp"

uint32_t ia_compute_outer_index(const Plane3D& p0)
{
    // Plane must not intersect tet at vertices.
    if (p0.f0 == 0 || p0.f1 == 0 || p0.f2 == 0 || p0.f3 == 0) return INVALID_INDEX;

    // To reuse the 2 plane lookup table, we assume the second plane is negative
    // on all tet vertices.
    size_t index = 0;
    if (p0.f0 > 0) index |= 1;
    if (p0.f1 > 0) index |= 4;
    if (p0.f2 > 0) index |= 16;
    if (p0.f3 > 0) index |= 64;

    return index;
}

uint32_t ia_compute_outer_index(const Plane3D& p0, const Plane3D& p1)
{
    // Plane must not intersect tet at vertices.
    if (p0.f0 == 0 || p0.f1 == 0 || p0.f2 == 0 || p0.f3 == 0) return INVALID_INDEX;
    if (p1.f0 == 0 || p1.f1 == 0 || p1.f2 == 0 || p1.f3 == 0) return INVALID_INDEX;

    size_t index = 0;
    if (p0.f0 > 0) index |= 1;
    if (p1.f0 > 0) index |= 2;
    if (p0.f1 > 0) index |= 4;
    if (p1.f1 > 0) index |= 8;
    if (p0.f2 > 0) index |= 16;
    if (p1.f2 > 0) index |= 32;
    if (p0.f3 > 0) index |= 64;
    if (p1.f3 > 0) index |= 128;

    return index;
}

uint32_t ia_compute_inner_index(uint32_t outer_index, const Plane3D& p0, const Plane3D& p1)
{
    std::bitset<2> v0 = outer_index & 3;
    std::bitset<2> v1 = (outer_index >> 2) & 3;
    std::bitset<2> v2 = (outer_index >> 4) & 3;
    std::bitset<2> v3 = (outer_index >> 6) & 3;

    size_t                index      = 0;
    size_t                edge_count = 0;
    std::array<double, 2> pp0, pp1;

    auto add_edge = [&](size_t i, size_t j) -> bool {
        pp0          = {(&p0.f0)[i], (&p0.f0)[j]};
        pp1          = {(&p1.f0)[i], (&p1.f0)[j]};
        const auto s = orient1d(pp0.data(), pp1.data());
        if (s == ORIENTATION_ZERO || s == ORIENTATION_INVALID) return false;

        if (s > 0) index |= (1 << edge_count);
        edge_count++;
        return true;
    };

    if ((v0 ^ v1).all())
        if (!add_edge(0, 1)) return INVALID_INDEX;
    if ((v0 ^ v2).all())
        if (!add_edge(0, 2)) return INVALID_INDEX;
    if ((v0 ^ v3).all())
        if (!add_edge(0, 3)) return INVALID_INDEX;
    if ((v1 ^ v2).all())
        if (!add_edge(1, 2)) return INVALID_INDEX;
    if ((v1 ^ v3).all())
        if (!add_edge(1, 3)) return INVALID_INDEX;
    if ((v2 ^ v3).all())
        if (!add_edge(2, 3)) return INVALID_INDEX;

    if (edge_count == 4) {
        assert(index != 6 && index != 9); // Impossible cases.
        if (index < 6) {
        } else if (index < 9) {
            index -= 1;                   // Skipping INVALID_INDEX case with index 6.
        } else {
            index -= 2;                   // Skipping INVALID_INDEX case with index 6 and 9.
        }
    }

    return index;
}