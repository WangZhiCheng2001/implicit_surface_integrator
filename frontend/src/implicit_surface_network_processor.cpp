#include <algorithm/glue_algorithm.hpp>

#include <extract_patch.hpp>
#include <implicit_arrangement.hpp>
#include <patch_connectivity.hpp>
#include <pair_faces.hpp>
#include <topology_ray_shooting.hpp>

#include <globals.hpp>
#include <internal_api.hpp>
#include <primitive_process.hpp>

#include <implicit_surface_network_processor.hpp>
#include "Eigen/src/Core/Matrix.h"

ImplicitSurfaceNetworkProcessor g_processor{};

void ImplicitSurfaceNetworkProcessor::preinit(const virtual_node_t& tree_node) noexcept
{
    auto           leaf_indices = blobtree_get_leaf_nodes(tree_node.main_index);
    virtual_node_t pointer      = tree_node;

    // 1. merge aabbs
    // 2. build mapping: primitive index -> leaf node index
    aabb_t scene_aabb{};
    leaf_index_of_primitive.resize(get_primitive_count());
    for (const auto& leaf_index : leaf_indices) {
        pointer.inner_index                      = leaf_index;
        const uint32_t primitive_index           = node_fetch_primitive_index(blobtree_get_node(pointer));
        leaf_index_of_primitive[primitive_index] = leaf_index;

        const auto& type = get_primitive_node(primitive_index).type;
        if (type != PRIMITIVE_TYPE_CONSTANT && type != PRIMITIVE_TYPE_PLANE) scene_aabb.extend(get_aabb(primitive_index));
    }

    // update background mesh using scene aabb
    // EDIT: scene aabb with a little margin
    this->background_mesh_manager.generate(scene_aabb.min - g_settings.scene_aabb_margin * Eigen::Vector3d::Ones(),
                                           scene_aabb.max + g_settings.scene_aabb_margin * Eigen::Vector3d::Ones());
}

void ImplicitSurfaceNetworkProcessor::clear() noexcept
{
    iso_vertices.clear();
    polygon_faces.clear();
    vertex_counts_of_face.clear();
}

solve_result_t ImplicitSurfaceNetworkProcessor::run(const virtual_node_t& tree_node) noexcept
{
    const auto& background_vertices = background_mesh_manager.get_vertices();
    const auto& background_indices  = background_mesh_manager.get_indices();
    if (background_vertices.empty() || background_indices.empty()) {
        std::cout << "Current network processor is runned before initialized!" << std::endl;
        return {};
    }

    const auto num_vert   = background_vertices.size();
    const auto num_tets   = background_indices.size();
    const auto num_funcs  = get_primitive_count(tree_node);
    const auto all_funcs  = get_all_primitive_index(tree_node);
    const auto all_offset = get_all_primitive_offset(tree_node);

    // temporary geometry results
    stl_vector_mp<polygon_face_t>          iso_faces{}; ///< Polygonal faces at the surface network mesh
    stl_vector_mp<stl_vector_mp<uint32_t>> patches{};   ///< A connected component of faces bounded by non-manifold edges
    stl_vector_mp<iso_edge_t>              iso_edges{}; ///< Edges at the surface network mesh
    stl_vector_mp<stl_vector_mp<uint32_t>> chains{};    ///< Chains of non-manifold edges
    stl_vector_mp<stl_vector_mp<uint32_t>> non_manifold_edges_of_vert{}; ///< Indices of non-manifold vertices
    stl_vector_mp<stl_vector_mp<uint32_t>>
        shells{}; ///< An array of shells. Each shell is a connected component consist of patches. Even patch index, 2*i,
                  ///< indicates patch i is consistently oriented with the shell. Odd patch index, 2*i+1, indicates patch i has
                  ///< opposite orientation with respect to the shell.
    stl_vector_mp<stl_vector_mp<uint32_t>>
        arrangement_cells{}; ///< A 3D region partitioned by the surface network; encoded by a vector of shell indices

    // compute function signs at vertices
    // EDIT: we only need to identify the sdf value is inside or on surface/outside
    // Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic> scalar_field_signs(num_funcs, num_vert);
    auto scalar_field_sign = [](double x) -> int8_t { return (x < 0) ? 1 : ((x > 0) ? -1 : 0); };
    stl_vector_mp<stl_vector_mp<double>> vertex_scalar_values(num_vert, stl_vector_mp<double>(num_funcs));
    stl_vector_mp<bool>                  is_positive_scalar_field_sign(num_funcs * num_vert, false);
    stl_vector_mp<bool>                  is_negative_scalar_field_sign(num_funcs * num_vert, false);
    stl_vector_mp<bool>                  is_degenerate_vertex(num_vert, false);
    bool                                 has_degenerate_vertex{};
    {
        g_timers_manager.push_timer("identify sdf signs");
        for (uint32_t i = 0; i < num_vert; ++i) {
            const auto& point = background_vertices[i];
            for (uint32_t j = 0; j < num_funcs; ++j) {
                vertex_scalar_values[i][j] = evaluate(all_funcs[j], point - all_offset[j]);
                const auto sign            = scalar_field_sign(vertex_scalar_values[i][j]);
                switch (sign) {
                    case -1: is_negative_scalar_field_sign[i * num_funcs + j] = true; break;
                    case 0:
                        is_degenerate_vertex[i] = true;
                        has_degenerate_vertex   = true;
                        break;
                    case 1:  is_positive_scalar_field_sign[i * num_funcs + j] = true; break;
                    default: break;
                }
            }
        }
        g_timers_manager.pop_timer("identify sdf signs");
    }

    // filter active functions in each tetrahedron
    // TODO: optimize this part by using SIMD and ranges::filter
    uint32_t                num_intersecting_tet = 0;
    stl_vector_mp<uint32_t> active_functions_in_tet{}; // active function indices in CRS vector format
    stl_vector_mp<uint32_t> start_index_of_tet{};
    {
        g_timers_manager.push_timer("filter active functions");
        active_functions_in_tet.reserve(num_tets);
        start_index_of_tet.reserve(num_tets + 1);
        start_index_of_tet.emplace_back(0);
        for (Eigen::Index i = 0; i < num_tets; ++i) {
            for (Eigen::Index j = 0; j < num_funcs; ++j) {
                uint32_t pos_count{}, neg_count{};
                for (uint32_t k = 0; k < 4; ++k) {
                    if (is_positive_scalar_field_sign[background_indices[i][k] * num_funcs + j]) pos_count++;
                    if (is_negative_scalar_field_sign[background_indices[i][k] * num_funcs + j]) neg_count++;
                }
                // if (scalar_field_signs(j, tet_ptr[k]) == 1) pos_count++;
                // tets[i].size() == 4, this means that the function is active in this tet
                if (pos_count < 4 && neg_count < 4) active_functions_in_tet.emplace_back(j);
            }
            if (active_functions_in_tet.size() > start_index_of_tet.back()) { ++num_intersecting_tet; }
            start_index_of_tet.emplace_back(static_cast<uint32_t>(active_functions_in_tet.size()));
        }
        g_timers_manager.pop_timer("filter active functions");
    }

    // compute arrangement in each tet
    // HINT: we skip robust test for this part for now
    stl_vector_mp<std::shared_ptr<arrangement_t>> cut_results{};
    uint32_t                                      num_1_func    = 0;
    uint32_t                                      num_2_func    = 0;
    uint32_t                                      num_more_func = 0;
    {
        // g_timers_manager.push_timer("implicit arrangements calculation in total");
        cut_results.reserve(num_intersecting_tet);
        stl_vector_mp<plane_t> planes{};
        planes.reserve(3);

        for (uint32_t i = 0; i < num_tets; ++i) {
            const auto start_index              = start_index_of_tet[i];
            const auto active_funcs_in_curr_tet = start_index_of_tet[i + 1] - start_index;
            if (active_funcs_in_curr_tet == 0) {
                cut_results.emplace_back(nullptr);
                continue;
            }
            g_timers_manager.push_timer(active_funcs_in_curr_tet == 1
                                            ? "implicit arrangements calculation (1 func)"
                                            : (active_funcs_in_curr_tet == 2
                                                   ? "implicit arrangments calculation (2 funcs)"
                                                   : "implicit arrangements calculation (>= 3 funcs)"));
            const auto& tet = background_indices[i];
            planes.clear();
            for (uint32_t j = 0; j < active_funcs_in_curr_tet; ++j) {
                const auto fid = active_functions_in_tet[start_index + j];
                planes.emplace_back(plane_t{-vertex_scalar_values[tet[0]][fid],
                                            -vertex_scalar_values[tet[1]][fid],
                                            -vertex_scalar_values[tet[2]][fid],
                                            -vertex_scalar_values[tet[3]][fid]});
            }
            cut_results.emplace_back(std::make_shared<arrangement_t>(std::move(compute_arrangement(planes))));
            switch (active_funcs_in_curr_tet) {
                case 1:
                    g_timers_manager.pop_timer("implicit arrangements calculation (1 func)");
                    ++num_1_func;
                    break;
                case 2:
                    g_timers_manager.pop_timer("implicit arrangments calculation (2 funcs)");
                    ++num_2_func;
                    break;
                default:
                    g_timers_manager.pop_timer("implicit arrangements calculation (>= 3 funcs)");
                    ++num_more_func;
                    break;
            }
        }
        // g_timers_manager.pop_timer("implicit arrangements calculation in total");
    }

    // extract arrangement mesh: combining results from all tets to produce a mesh
    // compute xyz coordinates of iso-vertices on the fly
    // HINT: vertices of faces are always oriented counterclockwise from the view of the positive side of the supporting plane
    // but since the sign is reversed, so that every face is always oriented clockwise when viewing outside
    stl_vector_mp<iso_vertex_t> iso_verts{};
    {
        g_timers_manager.push_timer("extract arrangement & iso mesh");
        extract_iso_mesh(num_1_func,
                         num_2_func,
                         num_more_func,
                         cut_results,
                         active_functions_in_tet,
                         start_index_of_tet,
                         background_mesh_manager.identity(),
                         vertex_scalar_values,
                         iso_vertices,
                         iso_verts,
                         iso_faces);
        g_timers_manager.pop_timer("extract arrangement & iso mesh");
    }

    //  compute iso-edges and edge-face connectivity
    stl_vector_mp<stl_vector_mp<uint32_t>> edges_of_iso_face{};
    {
        g_timers_manager.push_timer("compute iso-edge and edge-face connectivity");
        compute_patch_edges(iso_faces, edges_of_iso_face, iso_edges);
        g_timers_manager.pop_timer("compute iso-edge and edge-face connectivity");
    }

    // group iso-faces into patches
    // compute map: iso-face Id --> patch Id
    stl_vector_mp<uint32_t> patch_of_face(iso_faces.size());
    {
        g_timers_manager.push_timer("group iso-faces into patches");
        compute_patches(edges_of_iso_face, iso_edges, iso_faces, patches, patch_of_face);
        g_timers_manager.pop_timer("group iso-faces into patches");
    }

    // compute surface and volume integrals of patches
    stl_vector_mp<double> surf_int_of_patch{};
    stl_vector_mp<double> vol_int_of_patch{};
    {
        g_timers_manager.push_timer("compute surface and volume integrals of patches");
        surf_int_of_patch.reserve(patches.size());
        vol_int_of_patch.reserve(patches.size());
        for (const auto& face_of_patch_mapping : patches) {
            const auto& [surf_int, vol_int] = patch_integrator.integrate(iso_vertices, iso_faces, face_of_patch_mapping);
            surf_int_of_patch.emplace_back(std::move(surf_int));
            vol_int_of_patch.emplace_back(std::move(vol_int));
        }
        g_timers_manager.pop_timer("compute surface and volume integrals of patches");
    }

    // group non-manifold iso-edges into chains
    {
        g_timers_manager.push_timer("group non-manifold iso-edges into chains");
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
        g_timers_manager.pop_timer("group non-manifold iso-edges into chains");
    }

    // compute incident tets for degenerate vertices
    flat_hash_map_mp<uint32_t, stl_vector_mp<uint32_t>> incident_tets{};
    {
        g_timers_manager.push_timer("compute incident tets for degenerate vertices");
        if (has_degenerate_vertex) {
            for (uint32_t i = 0; i < num_tets; ++i) {
                for (uint32_t j = 0; j < 4; ++j) {
                    if (is_degenerate_vertex[background_indices[i][j]]) {
                        incident_tets[background_indices[i][j]].emplace_back(i);
                    }
                }
            }
        }
        g_timers_manager.pop_timer("compute incident tets for degenerate vertices");
    }

    // compute order of patches around chains
    // (patch i, 1) <--> 2i,  (patch i, -1) <--> 2i+1
    // compute half-patch adjacency list
    // stl_vector_mp<stl_vector_mp<half_patch_pair_t>> half_patch_pair_list{};
    stl_vector_mp<stl_vector_mp<uint32_t>> half_patch_adj_list(2 * patches.size());
    {
        g_timers_manager.push_timer("compute order of patches around chains");
        // half_patch_pair_list.resize(chains.size());
        // order iso-faces incident to each representative iso-edge
        for (uint32_t i = 0; i < chains.size(); i++) {
            // pick first iso-edge from each chain as representative
            const auto& iso_edge = iso_edges[chains[i][0]];
            // with degeneracy handling
            compute_patch_order(iso_edge,
                                background_indices,
                                iso_verts,
                                iso_faces,
                                cut_results,
                                active_functions_in_tet,
                                start_index_of_tet,
                                incident_tets,
                                patch_of_face,
                                half_patch_adj_list);
            // half_patch_pair_list[i]);
        }
        g_timers_manager.pop_timer("compute order of patches around chains");
    }

    // group patches into shells and components
    // each shell is represented as a list of half-patch indices
    // each component is represented as a list of patch indices
    stl_vector_mp<uint32_t>                shell_of_half_patch{};
    stl_vector_mp<stl_vector_mp<uint32_t>> components{};
    stl_vector_mp<uint32_t>                component_of_patch{};
    {
        g_timers_manager.push_timer("group patches into shells and components");
        compute_shells_and_components(half_patch_adj_list, shells, shell_of_half_patch, components, component_of_patch);
        g_timers_manager.pop_timer("group patches into shells and components");
    }

    // resolve nesting order, compute arrangement cells
    // an arrangement cell is represented by a list of bounding shells
    // get solve result by propagation
    solve_result_t result{};
    {
        g_timers_manager.push_timer("compute arrangement cells");
        if (components.size() == 1) { // no nesting problem, each shell is an arrangement cell
            {
                // only the -1 side shell is a valid arrangement cell
                auto& shells = arrangement_cells.emplace_back(1);
                shells.emplace_back(1);
                // arrangement_cells.reserve(shells.size());
                // for (uint32_t i = 0; i < shells.size(); ++i) {
                //     arrangement_cells.emplace_back(1);
                //     arrangement_cells.back()[0] = i;
                // }
            }
            result.mesh.vertices     = reinterpret_cast<const raw_vector3d_t*>(iso_vertices.data());
            result.mesh.num_vertices = static_cast<uint32_t>(iso_vertices.size());
            for (const auto& half_patch : shells[1]) {
                result.surf_int_result += surf_int_of_patch[half_patch / 2];
                result.vol_int_result  += -vol_int_of_patch[half_patch / 2];
                for (const auto& face_id : patches[half_patch / 2]) {
                    const auto& face = iso_faces[face_id];
                    polygon_faces.insert(polygon_faces.end(),
                                         std::make_move_iterator(polygon_faces.begin()),
                                         std::make_move_iterator(polygon_faces.end()));
                    vertex_counts_of_face.emplace_back(face.vertex_indices.size());
                }
            }
            result.mesh.faces         = polygon_faces.data();
            result.mesh.num_faces     = static_cast<uint32_t>(polygon_faces.size());
            result.mesh.vertex_counts = vertex_counts_of_face.data();
        } else { // resolve nesting order
            g_timers_manager.push_timer("arrangement cells: topo ray shooting");

            stl_vector_mp<std::pair<uint32_t, uint32_t>> shell_links{};
            topo_ray_shooting(background_mesh_manager.identity(),
                              cut_results,
                              iso_verts,
                              iso_faces,
                              patches,
                              patch_of_face,
                              shells,
                              shell_of_half_patch,
                              components,
                              component_of_patch,
                              shell_links);

            g_timers_manager.pop_timer("arrangement cells: topo ray shooting");

            // group shells into arrangement cells
            g_timers_manager.push_timer("arrangement cells: group shells");
            compute_arrangement_cells(static_cast<uint32_t>(shells.size()), shell_links, arrangement_cells);
            g_timers_manager.pop_timer("arrangement cells: group shells");

            // propagate solve result
            g_timers_manager.push_timer("arrangement cells: propagate solve result");
            result = std::move(patch_propagator.execute(tree_node,
                                                        leaf_index_of_primitive,
                                                        iso_vertices,
                                                        iso_faces,
                                                        patches,
                                                        surf_int_of_patch,
                                                        vol_int_of_patch,
                                                        arrangement_cells,
                                                        shell_of_half_patch,
                                                        shells,
                                                        polygon_faces,
                                                        vertex_counts_of_face));
            g_timers_manager.pop_timer("arrangement cells: propagate solve result");
        }
        g_timers_manager.pop_timer("compute arrangement cells");
    }

    result.success = true;
    return result;
}