#pragma once

#include <utils/fwd_types.hpp>

/// Compute iso-edges and edge-face connectivity
///
///@param[in] patch_faces            Patch faces taken from the kernel
///@param[out] edges_of_face            A list of face edges containing global edge index
///
///@param[out] patch_edges           Edges of the patch; containing two vertices, global face index, and local edge index
ISNP_API void compute_patch_edges(const stl_vector_mp<polygon_face_t>&    patch_faces,
                                  stl_vector_mp<stl_vector_mp<uint32_t>>& edges_of_face,
                                  stl_vector_mp<iso_edge_t>&              patch_edges);

/// Group `iso_faces` into patches for IA:
///
///
/// @param[in] edges_of_faces          the edges' indices of each face with the same orientation from the kernel.
/// @param[in] medg_edges            the list of edges on the patch.
/// @param[in] patch_faces            the list of faces on the patch. Each face contains an implicit function label.
///
///
/// @param[out] patches           output - the list of patches which contain a list of faces' indices.
ISNP_API void compute_patches(const stl_vector_mp<stl_vector_mp<uint32_t>>& edges_of_face,
                              const stl_vector_mp<iso_edge_t>&              patch_edges,
                              const stl_vector_mp<polygon_face_t>&          patch_faces,
                              stl_vector_mp<stl_vector_mp<uint32_t>>&       patches,
                              stl_vector_mp<uint32_t>&                      patch_of_face_mapping);

/// this should be correct, so we won't implement it for now.
// /// A validation of patch to function label through taking the majority vote of function labels on all the patch vertices
// ///
// ///@param[in] iso_verts             the list of vertices on the patch. Each vertex contains 1-3 implicit function labesl
// /// depending on its connectivity. This purely serves as a verification for the face labels above.
// ///
// ///@return Bool            True - passed the patch label checks / False - failed the check
// bool check_patch_label(const stl_vector_mp<stl_vector_mp<uint32_t>>& edges_of_face,
//                        const stl_vector_mp<IsoEdge>&                 patch_edges,
//                        const stl_vector_mp<PolygonFace>&             patch_faces,
//                        const stl_vector_mp<IsoVertex>                iso_verts,
//                        stl_vector_mp<stl_vector_mp<uint32_t>>&       patches,
//                        stl_vector_mp<uint32_t>&                      patch_function_label);

/// Group non-manifold iso-edges into chains
///
/// @param[in] patch_edges           As above
/// @param[out] non_manifold_edges_of_vert          Indices of non-manifold vertices
///
/// @param[out] chains          Chains of non-manifold edges; encoded by a vector of edge indices.
ISNP_API void compute_chains(const stl_vector_mp<iso_edge_t>&              patch_edges,
                             const stl_vector_mp<stl_vector_mp<uint32_t>>& non_manifold_edges_of_vert,
                             stl_vector_mp<stl_vector_mp<uint32_t>>&       chains);

/// compute shells and connected components of isosurfaces
/// each shell is a list of half-patches
/// each component is a list of patches
/// we also build maps: half-patch --> shell,  patch --> component
/// @param[in] half_patch_adj_list    The adjacency list of half-patches
///
/// @param[out] shell_of_patch          Map: half-patch --> shell
/// @param[out] component           Connected componeent represented as a list of patches
/// @param[out] component_of_patch          Map: patch --> component
ISNP_API void compute_shells_and_components(const stl_vector_mp<stl_vector_mp<uint32_t>>& half_patch_adj_list,
                                            stl_vector_mp<stl_vector_mp<uint32_t>>&       shells,
                                            stl_vector_mp<uint32_t>&                      shell_of_half_patch,
                                            stl_vector_mp<stl_vector_mp<uint32_t>>&       components,
                                            stl_vector_mp<uint32_t>&                      component_of_patch);

/// group shells into arrangement cells
///
///@param[in] num_shell         Shells number
///@param[in] shell_links           The connectivity of shells
///
///@param[out] arrangement_cells            Cells
ISNP_API void compute_arrangement_cells(uint32_t                                            num_shell,
                                        const stl_vector_mp<std::pair<uint32_t, uint32_t>>& shell_links,
                                        stl_vector_mp<stl_vector_mp<uint32_t>>&             arrangement_cells);