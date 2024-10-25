#pragma once

#include <stdint.h>
#include <stdbool.h>

#include <macros.h>

/**
 * A plane is defined by the barycentric plane equation:
 *     f0 * b0 + f1 * b1 + f2 * b2 + f3 * b3 = 0 // For 3D
 *     f0 * b0 + f1 * b1 + f2 * b2 = 0           // For 2D
 * where the b's are the barycentric variables, and f's are the
 * plane equation coefficients.  We store the f's for each plane.
 */
struct Plane2D {
    double f0, f1, f2;
};

struct Plane3D {
    double f0, f1, f2, f3;
};

/**
 * A point is represented as the intersection of planes.  We store the index
 * of the plane here.
 */
struct Point2D {
    uint32_t i0, i1;
};

struct Point3D {
    uint32_t i0, i1, i2;
};

/**
 * A self-contained data structure for 2D or 3D arrangement representation.
 */
struct Arrangement2D {
    /* vertex descriptor */
    Point2D*   points;
    /* edge descriptor */
    uint32_t*  vertices; ///< An ordered list of boundary vertices. Edge is oriented such that the positive side of supporting
                         ///< line is one the right.
    // an 2D edge has restrictly 2 vertices
    uint32_t*  supporting_lines; ///< A set of supporting lines' indices for each edge.
    uint32_t*  positive_cells;   ///<  A set of positive side cells' indices for each edge.
    uint32_t*  negative_cells;   ///<  A set of negative side cells' indices for each edge.
    /* cell descriptor */
    uint32_t** edges;            ///< A set of boundary edge indices in no particular order.
    uint32_t*  face_edges_count; ///< The number of edges in each cell.

    /* Note: the following structure is only non-empty if input planes contain duplicates. */
    uint32_t*  unique_plane_indices;
    uint32_t** unique_planes;
    uint32_t*  unique_plane_count;
    bool*      unique_plane_orientations;

    uint32_t num_vertices;
    uint32_t num_edges;
    uint32_t num_faces;
    uint32_t num_unique_planes;
};

struct Arrangement3D {
    /* vertex descriptor */
    Point3D*   points;
    /* face descriptor */
    uint32_t** vertices; ///< An ordered list of boundary vertices. The face is always oriented counterclockwise when viewed
                         ///< from the positive side of the supporting plane.
    uint32_t*  face_vertices_count; ///< The number of vertices in each edge.
    uint32_t*  supporting_planes;   ///< A set of supporting planes' indices for each edge.
    uint32_t*  positive_cells;      ///<  A set of positive side cells' indices for each edge.
    uint32_t*  negative_cells;      ///<  A set of negative side cells' indices for each edge.
    /* cell descriptor */
    uint32_t** faces;            ///< A set of boundary face indices in no particular order.
    uint32_t*  cell_faces_count; ///< The number of edges in each cell.

    /* Note: the following structure is only non-empty if input planes contain duplicates. */
    uint32_t*  unique_plane_indices;
    uint32_t** unique_planes;
    uint32_t*  unique_plane_count;
    bool*      unique_plane_orientations;

    uint32_t num_vertices;
    uint32_t num_faces;
    uint32_t num_cells;
    uint32_t num_unique_planes;
};

struct Arrangement2DResult {
    Arrangement2D arrangement;
    bool          is_runtime_computed;
};

struct Arrangement3DResult {
    Arrangement3D arrangement;
    bool          is_runtime_computed;
};

EXTERN_C API bool                load_lut();
EXTERN_C API void                lut_print_test();
EXTERN_C API Arrangement2DResult compute_arrangement_2d(const Plane2D* planes, const uint32_t num_planes);
EXTERN_C API Arrangement3DResult compute_arrangement_3d(const Plane3D* planes, const uint32_t num_planes);
EXTERN_C API void                free_arrangement_2d(Arrangement2D* arr);
EXTERN_C API void                free_arrangement_3d(Arrangement3D* arr);