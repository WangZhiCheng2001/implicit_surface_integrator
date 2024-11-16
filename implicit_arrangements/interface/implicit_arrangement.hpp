#pragma once

#include <tbb/tbb.h>

#include <macros.h>
#include <container/small_vector.hpp>

/**
 * A plane is defined by the barycentric plane equation:
 *     f0 * b0 + f1 * b1 + f2 * b2 + f3 * b3 = 0 // For 3D
 * where the b's are the barycentric variables, and f's are the
 * plane equation coefficients.  We store the f's for each plane.
 */
using plane_t = std::array<double, 4>;

/**
 * A point is represented as the intersection of planes.  We store the index
 * of the plane here.
 */
using point_t = std::array<uint32_t, 3>;

template <typename T>
using stl_vector_mp = std::vector<T, tbb::tbb_allocator<T>>;
template <typename T>
using tbb_vector_mp                     = tbb::concurrent_vector<T, tbb::tbb_allocator<T>>;
static constexpr uint32_t INVALID_INDEX = std::numeric_limits<uint32_t>::max();

struct arrangement_t {
    /* vertex descriptor */
    stl_vector_mp<point_t> vertices{};

    /* face descriptor */
    struct face_descriptor {
        stl_vector_mp<uint32_t> vertices{}; ///< An ordered list of boundary vertices. The face is always oriented
                                            ///< counterclockwise when viewed from the positive side of the supporting plane.
        uint32_t                supporting_plane{INVALID_INDEX}; ///< A set of supporting planes' indices for each edge.
        uint32_t                positive_cell{INVALID_INDEX};    ///< A set of positive side cells' indices for each edge.
        uint32_t                negative_cell{INVALID_INDEX};    ///< A set of negative side cells' indices for each edge.
    };

    stl_vector_mp<face_descriptor> faces{};                      ///< A set of boundary vertex indices in no particular order.

    /* cell descriptor */
    struct cell_descriptor {
        stl_vector_mp<uint32_t> faces{};
    };

    stl_vector_mp<cell_descriptor> cells{}; ///< A set of boundary face indices in no particular order.

    /* Note: the following structure is only non-empty if input planes contain duplicates. */
    stl_vector_mp<uint32_t>                unique_plane_indices{};
    stl_vector_mp<stl_vector_mp<uint32_t>> unique_planes{};
    stl_vector_mp<bool>                    unique_plane_orientations{};
};

IA_API bool          load_lut();
IA_API void          lut_print_test();
IA_API arrangement_t compute_arrangement(const tbb_vector_mp<plane_t>& planes);