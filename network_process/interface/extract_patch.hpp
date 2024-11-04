#pragma once

#include <utils/fwd_types.hpp>

// extract iso-mesh (topology only)
ISNP_API void extract_iso_mesh(uint32_t                                           num_1_func,
                               uint32_t                                           num_2_func,
                               uint32_t                                           num_more_func,
                               const stl_vector_mp<arrangement_t>&                cut_results,
                               const stl_vector_mp<uint32_t>&                     cut_result_index,
                               const stl_vector_mp<uint32_t>&                     func_in_tet,
                               const stl_vector_mp<uint32_t>&                     start_index_of_tet,
                               const stl_vector_mp<tetrahedron_vertex_indices_t>& tets,
                               stl_vector_mp<iso_vertex_t>&                       iso_verts,
                               stl_vector_mp<polygon_face_t>&                     iso_faces);

// given the list of vertex indices of a face, return the unique key of the face: (the smallest vert Id,
// second-smallest vert Id, the largest vert Id) assume: face_verts is a list of non-duplicate natural
// numbers, with at least three elements.
ISNP_API void compute_iso_face_key(const stl_vector_mp<uint32_t>& face_verts, pod_key_t<3>& key);

// compute xyz coordinates of iso-vertices
ISNP_API void compute_iso_vert_xyz(const stl_vector_mp<iso_vertex_t>&       iso_verts,
                                   const Eigen::Ref<const Eigen::MatrixXd>& func_vals,
                                   const stl_vector_mp<raw_point_t>&        pts,
                                   stl_vector_mp<raw_point_t>&              iso_pts);

// compute barycentric coordinate of Point (intersection of three planes)
// Point in tet cell
template <typename Scalar>
inline void compute_barycentric_coords(const std::array<Scalar, 4>& plane1,
                                       const std::array<Scalar, 4>& plane2,
                                       const std::array<Scalar, 4>& plane3,
                                       std::array<Scalar, 4>&       bary_coords)
{
    Scalar n1 = plane1[3] * (plane2[2] * plane3[1] - plane2[1] * plane3[2])
                + plane1[2] * (plane2[1] * plane3[3] - plane2[3] * plane3[1])
                + plane1[1] * (plane2[3] * plane3[2] - plane2[2] * plane3[3]);
    Scalar n2 = plane1[3] * (plane2[0] * plane3[2] - plane2[2] * plane3[0])
                + plane1[2] * (plane2[3] * plane3[0] - plane2[0] * plane3[3])
                + plane1[0] * (plane2[2] * plane3[3] - plane2[3] * plane3[2]);
    Scalar n3 = plane1[3] * (plane2[1] * plane3[0] - plane2[0] * plane3[1])
                + plane1[1] * (plane2[0] * plane3[3] - plane2[3] * plane3[0])
                + plane1[0] * (plane2[3] * plane3[1] - plane2[1] * plane3[3]);
    Scalar n4 = plane1[2] * (plane2[0] * plane3[1] - plane2[1] * plane3[0])
                + plane1[1] * (plane2[2] * plane3[0] - plane2[0] * plane3[2])
                + plane1[0] * (plane2[1] * plane3[2] - plane2[2] * plane3[1]);
    Scalar d       = n1 + n2 + n3 + n4;
    //
    bary_coords[0] = n1 / d;
    bary_coords[1] = n2 / d;
    bary_coords[2] = n3 / d;
    bary_coords[3] = n4 / d;
}

// Point on tet face
template <typename Scalar>
inline void compute_barycentric_coords(const std::array<Scalar, 3>& plane1,
                                       const std::array<Scalar, 3>& plane2,
                                       std::array<Scalar, 3>&       bary_coords)
{
    Scalar n1      = plane1[2] * plane2[1] - plane1[1] * plane2[2];
    Scalar n2      = plane1[0] * plane2[2] - plane1[2] * plane2[0];
    Scalar n3      = plane1[1] * plane2[0] - plane1[0] * plane2[1];
    Scalar d       = n1 + n2 + n3;
    //
    bary_coords[0] = n1 / d;
    bary_coords[1] = n2 / d;
    bary_coords[2] = n3 / d;
}

// Point on tet edge
template <typename Scalar>
inline void compute_barycentric_coords(Scalar f1, Scalar f2, std::array<Scalar, 2>& bary_coords)
{
    bary_coords[0] = f2 / (f2 - f1);
    bary_coords[1] = 1 - bary_coords[0];
}