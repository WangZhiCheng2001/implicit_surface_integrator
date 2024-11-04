#include "extract_patch.hpp"
#include "patch_connectivity.hpp"
#include "pair_faces.hpp"
#include "topology_ray_shooting.hpp"
#include "timer/scoped_timer.hpp"

#include <implicit_surface_network_processor.hpp>

bool ImplicitSurfaceNetworkProcessor::run(labelled_timers_manager& timers_manager)
{
    if (background_mesh.vertices.empty() || background_mesh.indices.empty()) {
        std::cout << "Current network processor is runned before initialized!" << std::endl;
        return false;
    }

    const auto num_vert  = background_mesh.vertices.size();
    const auto num_tets  = background_mesh.indices.size();
    const auto num_funcs = sdf_scalar_field.rows();

    // compute function signs at vertices
    // EDIT: we only need to identify the sdf value is inside or on surface/outside
    // Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic> scalar_field_signs(num_funcs, num_vert);
    auto                scalar_field_sign = [](double x) -> int8_t { return (x > 0) ? 1 : ((x < 0) ? -1 : 0); };
    stl_vector_mp<bool> is_positive_scalar_field_sign(num_funcs * num_vert, false);
    stl_vector_mp<bool> is_negative_scalar_field_sign(num_funcs * num_vert, false);
    stl_vector_mp<bool> is_degenerate_vertex(num_vert, false);
    bool                has_degenerate_vertex{};
    {
        timers_manager.push_timer("identify sdf signs");
        for (size_t i = 0; i < sdf_scalar_field.size(); ++i) {
            const auto sign = scalar_field_sign(*(sdf_scalar_field.data() + i));
            switch (sign) {
                case -1: is_negative_scalar_field_sign[i] = true; break;
                case 0:
                    is_degenerate_vertex[i] = true;
                    has_degenerate_vertex   = true;
                    break;
                case 1:  is_positive_scalar_field_sign[i] = true; break;
                default: break;
            }
        }
        timers_manager.pop_timer("identify sdf signs");
    }

    // filter active functions in each tetrahedron
    // TODO: optimize this part by using SIMD and ranges::filter
    uint32_t                num_intersecting_tet = 0;
    stl_vector_mp<uint32_t> func_in_tet{}; // active function indices in CRS vector format
    stl_vector_mp<uint32_t> start_index_of_tet{};
    {
        timers_manager.push_timer("filter active functions");
        func_in_tet.reserve(num_tets);
        start_index_of_tet.reserve(num_tets + 1);
        start_index_of_tet.emplace_back(0);
        for (Eigen::Index i = 0; i < num_tets; ++i) {
            for (Eigen::Index j = 0; j < num_funcs; ++j) {
                uint32_t pos_count{}, neg_count{};
                for (uint32_t k = 0; k < 4; ++k) {
                    if (is_positive_scalar_field_sign[background_mesh.indices[i][k] * num_funcs + j]) pos_count++;
                    if (is_negative_scalar_field_sign[background_mesh.indices[i][k] * num_funcs + j]) neg_count++;
                }
                // if (scalar_field_signs(j, tet_ptr[k]) == 1) pos_count++;
                // tets[i].size() == 4, this means that the function is active in this tet
                if (pos_count < 4 && neg_count < 4) func_in_tet.emplace_back(j);
            }
            if (func_in_tet.size() > start_index_of_tet.back()) { ++num_intersecting_tet; }
            start_index_of_tet.emplace_back(static_cast<uint32_t>(func_in_tet.size()));
        }
        timers_manager.pop_timer("filter active functions");
    }

    // compute arrangement in each tet
    // HINT: we skip robust test for this part for now
    stl_vector_mp<arrangement_t> cut_results{};
    stl_vector_mp<uint32_t>      cut_result_index{};
    uint32_t                     num_1_func    = 0;
    uint32_t                     num_2_func    = 0;
    uint32_t                     num_more_func = 0;
    {
        // timers_manager.push_timer("implicit arrangements calculation in total");
        cut_results.reserve(num_intersecting_tet);
        cut_result_index.reserve(num_tets);
        stl_vector_mp<plane_t> planes{};
        planes.reserve(3);
        try {
            for (uint32_t i = 0; i < num_tets; ++i) {
                const auto start_index              = start_index_of_tet[i];
                const auto active_funcs_in_curr_tet = start_index_of_tet[i + 1] - start_index;
                if (active_funcs_in_curr_tet == 0) {
                    cut_result_index.emplace_back(invalid_index);
                    continue;
                }
                timers_manager.push_timer(active_funcs_in_curr_tet == 1
                                              ? "implicit arrangements calculation (1 func)"
                                              : (active_funcs_in_curr_tet == 2
                                                     ? "implicit arrangments calculation (2 funcs)"
                                                     : "implicit arrangements calculation (>= 3 funcs)"));
                const auto& tet = background_mesh.indices[i];
                planes.clear();
                for (uint32_t j = 0; j < active_funcs_in_curr_tet; ++j) {
                    const auto fid = func_in_tet[start_index + j];
                    planes.emplace_back(plane_t{sdf_scalar_field(fid, tet[0]),
                                                sdf_scalar_field(fid, tet[1]),
                                                sdf_scalar_field(fid, tet[2]),
                                                sdf_scalar_field(fid, tet[3])});
                }
                cut_result_index.emplace_back(static_cast<uint32_t>(cut_results.size()));
                auto result = compute_arrangement(planes);
                cut_results.emplace_back(std::move(result));
                switch (active_funcs_in_curr_tet) {
                    case 1:
                        timers_manager.pop_timer("implicit arrangements calculation (1 func)");
                        ++num_1_func;
                        break;
                    case 2:
                        timers_manager.pop_timer("implicit arrangments calculation (2 funcs)");
                        ++num_2_func;
                        break;
                    default:
                        timers_manager.pop_timer("implicit arrangements calculation (>= 3 funcs)");
                        ++num_more_func;
                        break;
                }
            }
        } catch (const std::exception& e) {
            std::cout << "Error: " << e.what() << std::endl;
            return false;
        }
        // timers_manager.pop_timer("implicit arrangements calculation in total");
    }

    // extract arrangement mesh: combining results from all tets to produce a mesh
    stl_vector_mp<iso_vertex_t> iso_verts{};
    {
        timers_manager.push_timer("extract arrangement mesh");
        extract_iso_mesh(num_1_func,
                         num_2_func,
                         num_more_func,
                         cut_results,
                         cut_result_index,
                         func_in_tet,
                         start_index_of_tet,
                         background_mesh.indices,
                         iso_verts,
                         iso_faces);
        timers_manager.pop_timer("extract arrangement mesh");
    }

    // compute xyz coordinates of iso-vertices
    {
        timers_manager.push_timer("compute iso-vertex coordinates");
        compute_iso_vert_xyz(iso_verts, sdf_scalar_field, background_mesh.vertices, iso_vertices);
        timers_manager.pop_timer("compute iso-vertex coordinates");
    }

    //  compute iso-edges and edge-face connectivity
    stl_vector_mp<stl_vector_mp<uint32_t>> edges_of_iso_face{};
    {
        timers_manager.push_timer("compute iso-edge and edge-face connectivity");
        compute_patch_edges(iso_faces, edges_of_iso_face, iso_edges);
        timers_manager.pop_timer("compute iso-edge and edge-face connectivity");
    }

    // group iso-faces into patches
    {
        timers_manager.push_timer("group iso-faces into patches");
        compute_patches(edges_of_iso_face, iso_edges, iso_faces, patches, patch_function_labels);
        timers_manager.pop_timer("group iso-faces into patches");
    }

    // compute map: iso-face Id --> patch Id
    stl_vector_mp<uint32_t> patch_of_face(iso_faces.size());
    {
        timers_manager.push_timer("compute map: iso-face Id --> patch Id");
        for (uint32_t i = 0; i < patches.size(); ++i) {
            for (const auto& fId : patches[i]) { patch_of_face[fId] = i; }
        }
        timers_manager.pop_timer("compute map: iso-face Id --> patch Id");
    }

    // group non-manifold iso-edges into chains
    {
        timers_manager.push_timer("group non-manifold iso-edges into chains");
        non_manifold_edges_of_vert.resize(iso_verts.size());
        // get incident non-manifold edges for iso-vertices
        for (uint32_t i = 0; i < iso_edges.size(); i++) {
            if (iso_edges[i].headers.size() > 2) { // non-manifold edge (not a boundary edge)
                // there is only one patch incident to a boundary edge,
                // so there is no need to figure out the "order" of patches around a boundary
                // edge
                non_manifold_edges_of_vert[iso_edges[i].v1].emplace_back(i);
                non_manifold_edges_of_vert[iso_edges[i].v2].emplace_back(i);
            }
        }
        // group non-manifold iso-edges into chains
        compute_chains(iso_edges, non_manifold_edges_of_vert, chains);
        timers_manager.pop_timer("group non-manifold iso-edges into chains");
    }

    // compute incident tets for degenerate vertices
    flat_hash_map_mp<uint32_t, stl_vector_mp<uint32_t>> incident_tets{};
    {
        timers_manager.push_timer("compute incident tets for degenerate vertices");
        if (has_degenerate_vertex) {
            for (uint32_t i = 0; i < num_tets; ++i) {
                for (uint32_t j = 0; j < 4; ++j) {
                    if (is_degenerate_vertex[background_mesh.indices[i][j]]) {
                        incident_tets[background_mesh.indices[i][j]].emplace_back(i);
                    }
                }
            }
        }
        timers_manager.pop_timer("compute incident tets for degenerate vertices");
    }

    // compute order of patches around chains
    // pair<uint32_t, int8_t> : pair (iso-face index, iso-face orientation)
    stl_vector_mp<stl_vector_mp<half_face_pair_t>>  half_face_pair_list{};
    stl_vector_mp<stl_vector_mp<half_patch_pair_t>> half_patch_pair_list{};
    {
        timers_manager.push_timer("compute order of patches around chains");
        half_face_pair_list.resize(chains.size());
        half_patch_pair_list.resize(chains.size());
        // pick representative iso-edge from each chain
        stl_vector_mp<uint32_t> chain_representatives(chains.size());
        for (uint32_t i = 0; i < chains.size(); i++) { chain_representatives[i] = chains[i][0]; }
        // order iso-faces incident to each representative iso-edge
        for (uint32_t i = 0; i < chain_representatives.size(); i++) {
            const auto& iso_edge = iso_edges[chain_representatives[i]];
            // with degeneracy handling
            compute_face_order(iso_edge,
                               background_mesh.indices,
                               iso_verts,
                               iso_faces,
                               cut_results,
                               cut_result_index,
                               func_in_tet,
                               start_index_of_tet,
                               incident_tets,
                               half_face_pair_list[i]);
        }
        // replace iso-face indices by patch indices
        for (uint32_t i = 0; i < half_face_pair_list.size(); i++) {
            half_patch_pair_list[i].resize(half_face_pair_list[i].size());
            for (uint32_t j = 0; j < half_face_pair_list[i].size(); j++) {
                half_patch_pair_list[i][j] = half_patch_pair_t{
                    half_patch_t{patch_of_face[half_face_pair_list[i][j].first.index],
                                 half_face_pair_list[i][j].first.orientation },
                    half_patch_t{patch_of_face[half_face_pair_list[i][j].second.index],
                                 half_face_pair_list[i][j].second.orientation}
                };
            }
        }
        timers_manager.pop_timer("compute order of patches around chains");
    }

    // group patches into shells and components
    // each shell is represented as a list of half-patch indices
    // each component is represented as a list of patch indices
    stl_vector_mp<uint32_t>                shell_of_half_patch{};
    stl_vector_mp<stl_vector_mp<uint32_t>> components{};
    stl_vector_mp<uint32_t>                component_of_patch{};
    {
        timers_manager.push_timer("group patches into shells and components");
        compute_shells_and_components(static_cast<uint32_t>(patches.size()),
                                      half_patch_pair_list,
                                      shells,
                                      shell_of_half_patch,
                                      components,
                                      component_of_patch);
        timers_manager.pop_timer("group patches into shells and components");
    }

    // resolve nesting order, compute arrangement cells
    // an arrangement cell is represented by a list of bounding shells
    {
        timers_manager.push_timer("compute arrangement cells");
        if (components.size() < 2) { // no nesting problem, each shell is an arrangement cell
            arrangement_cells.reserve(shells.size());
            for (uint32_t i = 0; i < shells.size(); ++i) {
                arrangement_cells.emplace_back(1);
                arrangement_cells.back()[0] = i;
            }
        } else { // resolve nesting order
            timers_manager.push_timer("arrangement cells: topo ray shooting");
            topo_ray_shooting(background_mesh,
                              cut_results,
                              cut_result_index,
                              iso_verts,
                              iso_faces,
                              patches,
                              patch_of_face,
                              shells,
                              shell_of_half_patch,
                              components,
                              component_of_patch,
                              arrangement_cells);
            timers_manager.pop_timer("arrangement cells: topo ray shooting");
        }
        timers_manager.pop_timer("compute arrangement cells");
    }

    // fetching function labels (True, False) for each cell
    stl_vector_mp<bool> sample_function_label(is_positive_scalar_field_sign.begin(),
                                              is_positive_scalar_field_sign.begin() + num_funcs + 1);
    cell_function_labels = sign_propagation(arrangement_cells,
                                            shell_of_half_patch,
                                            shells,
                                            patch_function_labels,
                                            num_funcs,
                                            sample_function_label);

    return true;
}