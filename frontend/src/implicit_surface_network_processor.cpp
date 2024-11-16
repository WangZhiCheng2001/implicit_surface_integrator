#include <algorithm/glue_algorithm.hpp>

#include <extract_patch.hpp>
#include <implicit_arrangement.hpp>
#include <patch_connectivity.hpp>
#include <pair_faces.hpp>
#include <topology_ray_shooting.hpp>

#include <internal_api.hpp>

#include <implicit_surface_network_processor.hpp>

ImplicitSurfaceNetworkProcessor g_processor{};

void ImplicitSurfaceNetworkProcessor::update_background_mesh(const Eigen::Ref<const raw_point_t>& aabb_min,
                                                             const Eigen::Ref<const raw_point_t>& aabb_max) noexcept
{
    this->background_mesh_manager.generate(aabb_min, aabb_max);
}

void ImplicitSurfaceNetworkProcessor::clear() noexcept
{
    iso_vertices.clear();
    iso_faces.clear();
    patches.clear();
    patch_function_labels.clear();
    iso_edges.clear();
    non_manifold_edges_of_vert.clear();
    shells.clear();
    arrangement_cells.clear();
    cell_function_labels.clear();
    surf_int_of_patch.clear();
    vol_int_of_patch.clear();
}

bool ImplicitSurfaceNetworkProcessor::run(labelled_timers_manager& timers_manager) noexcept
{
    const auto& background_vertices = background_mesh_manager.get_vertices();
    const auto& background_indices  = background_mesh_manager.get_indices();
    if (background_vertices.empty() || background_indices.empty()) {
        std::cout << "Current network processor is runned before initialized!" << std::endl;
        return false;
    }

    const auto  num_vert   = background_vertices.size();
    const auto  num_tets   = background_indices.size();
    const auto& primitives = get_primitives();
    const auto  num_funcs  = primitives.size();

    // composed initialization stage:
    // 1. compute function signs at vertices
    // 2. filter active functions in each tetrahedron
    // 3. compute arrangement in each tet (skip robust test)
    // 4. compute incident tets for degenerate vertices
    // stl_vector_mp<bool>                           sample_function_label(num_funcs, false);
    stl_vector_mp<stl_vector_mp<double>>          vertex_scalar_values(num_vert, stl_vector_mp<double>(num_funcs));
    stl_vector_mp<uint32_t>                       active_functions_in_tet{}; // active function indices in CRS vector format
    stl_vector_mp<uint32_t>                       start_index_of_tet{};
    stl_vector_mp<std::shared_ptr<arrangement_t>> cut_results(num_tets);
    std::atomic_uint32_t                          num_1_func{};
    std::atomic_uint32_t                          num_2_func{};
    std::atomic_uint32_t                          num_more_func{};
    parallel_flat_hash_map_mp<uint32_t, stl_vector_mp<uint32_t>> incident_tets{};
    {
        struct vertex_info_t {
            // iff conmpact, then we should have 4 bits for each function sign, that is 1 bit for positive and negative
            // and other 3 bits for counter (at most -+4 for every sign)
            // as a simplified version, we use int8_t for now
            // iff needed to use compact version, we may implement packed_int4_2_t
            stl_vector_mp<int8_t> signs{};
        };

        stl_vector_mp<vertex_info_t>  vertex_info(num_vert, vertex_info_t{stl_vector_mp<int8_t>(num_funcs)});
        stl_vector_mp<std::once_flag> vertex_sign_constructed(num_vert);
        active_functions_in_tet.resize(num_tets * num_funcs);
        start_index_of_tet.resize(num_tets + 1, 0);
        std::atomic_uint32_t curr_active_func_index{0};

        auto evaluate_func = [&](auto index, const raw_point_t& point) {
            switch (primitives[index].type) {
                case PRIMITIVE_TYPE_CONSTANT: return evaluate(*(const constant_descriptor_t*)primitives[index].desc, point);
                case PRIMITIVE_TYPE_PLANE:    return evaluate(*(const plane_descriptor_t*)primitives[index].desc, point);
                case PRIMITIVE_TYPE_SPHERE:   return evaluate(*(const sphere_descriptor_t*)primitives[index].desc, point);
                case PRIMITIVE_TYPE_CYLINDER: return evaluate(*(const cylinder_descriptor_t*)primitives[index].desc, point);
                case PRIMITIVE_TYPE_CONE:     return evaluate(*(const cone_descriptor_t*)primitives[index].desc, point);
                case PRIMITIVE_TYPE_BOX:      return evaluate(*(const box_descriptor_t*)primitives[index].desc, point);
                case PRIMITIVE_TYPE_MESH:     return evaluate(*(const mesh_descriptor_t*)primitives[index].desc, point);
                case PRIMITIVE_TYPE_EXTRUDE:  return evaluate(*(const extrude_descriptor_t*)primitives[index].desc, point);
            }
        };

        auto scalar_field_sign       = [](double x) -> int8_t { return (x > 0) ? 1 : ((x < 0) ? -1 : 0); };
        auto get_or_init_vertex_info = [&, this](uint32_t vert_index, uint32_t tet_index) {
            std::call_once(vertex_sign_constructed[vert_index], [&, this] {
                const auto& vertex        = background_vertices[vert_index];
                auto&       scalar_values = vertex_scalar_values[vert_index];
                for (Eigen::Index i = 0; i < num_funcs; ++i) {
                    scalar_values[i]                 = evaluate_func(i, vertex);
                    vertex_info[vert_index].signs[i] = scalar_field_sign(scalar_values[i]);
                    if (vertex_info[vert_index].signs[i] == 0) {
                        incident_tets.lazy_emplace_l(
                            vert_index,
                            [&](decltype(incident_tets)::value_type& value) { value.second.emplace_back(tet_index); },
                            [&](const decltype(incident_tets)::constructor& ctor) {
                                ctor(vert_index, stl_vector_mp<uint32_t>{tet_index});
                            });
                    }
                }
            });

            return std::make_tuple(&vertex_info[vert_index], &vertex_scalar_values[vert_index]);
        };

        timers_manager.push_timer("composed init");

        tbb::parallel_for(size_t{0}, num_tets, [&](size_t i) {
            tbb_vector_mp<plane_t> planes{};
            planes.reserve(3);

            const auto& indices = background_indices[i];
            const auto& v0      = get_or_init_vertex_info(indices[0], i);
            const auto& v1      = get_or_init_vertex_info(indices[1], i);
            const auto& v2      = get_or_init_vertex_info(indices[2], i);
            const auto& v3      = get_or_init_vertex_info(indices[3], i);
            const auto& vi0     = *std::get<0>(v0);
            const auto& vi1     = *std::get<0>(v1);
            const auto& vi2     = *std::get<0>(v2);
            const auto& vi3     = *std::get<0>(v3);
            const auto& vs0     = *std::get<1>(v0);
            const auto& vs1     = *std::get<1>(v1);
            const auto& vs2     = *std::get<1>(v2);
            const auto& vs3     = *std::get<1>(v3);

            start_index_of_tet[i + 1] = static_cast<uint32_t>(curr_active_func_index.load(std::memory_order_acquire));
            auto& index               = start_index_of_tet[i + 1];
            algorithm::for_loop<algorithm::ExecutionPolicySelector::simd_only>(size_t{0}, num_funcs, [&](size_t j) {
                if (auto sign = vi0.signs[j] + vi1.signs[j] + vi2.signs[j] + vi3.signs[j]; -4 < sign && sign < 4) {
                    index                              = curr_active_func_index.fetch_add(1, std::memory_order_acq_rel) + 1;
                    active_functions_in_tet[index - 1] = static_cast<uint32_t>(j);
                    planes.emplace_back(plane_t{vs0[j], vs1[j], vs2[j], vs3[j]});
                }
            });

            switch (planes.size()) {
                case 0: break;
                case 1:
                    num_1_func++;
                    cut_results[i] = std::make_shared<arrangement_t>(std::move(compute_arrangement(planes)));
                    break;
                case 2:
                    num_2_func++;
                    cut_results[i] = std::make_shared<arrangement_t>(std::move(compute_arrangement(planes)));
                    break;
                default:
                    num_more_func++;
                    cut_results[i] = std::make_shared<arrangement_t>(std::move(compute_arrangement(planes)));
                    break;
            }
        });

        timers_manager.pop_timer("composed init");
    }

    // extract arrangement mesh: combining results from all tets to produce a mesh
    // compute xyz coordinates of iso-vertices on the fly
    stl_vector_mp<iso_vertex_t> iso_verts{};
    {
        timers_manager.push_timer("extract arrangement & iso mesh");
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
        timers_manager.pop_timer("extract arrangement & iso mesh");
    }

    //  compute iso-edges and edge-face connectivity
    stl_vector_mp<stl_vector_mp<uint32_t>> edges_of_iso_face{};
    {
        timers_manager.push_timer("compute iso-edge and edge-face connectivity");
        compute_patch_edges(iso_faces, edges_of_iso_face, iso_edges);
        timers_manager.pop_timer("compute iso-edge and edge-face connectivity");
    }

    // group iso-faces into patches
    // compute map: iso-face Id --> patch Id
    stl_vector_mp<uint32_t> patch_of_face(iso_faces.size());
    {
        timers_manager.push_timer("group iso-faces into patches");
        compute_patches(edges_of_iso_face, iso_edges, iso_faces, patches, patch_function_labels, patch_of_face);
        timers_manager.pop_timer("group iso-faces into patches");
    }

    {
        timers_manager.push_timer("compute surface and volume integrals of patches");
        surf_int_of_patch.reserve(patches.size());
        vol_int_of_patch.reserve(patches.size());
        for (const auto& face_of_patch_mapping : patches) {
            const auto& [surf_int, vol_int] = patch_integrator.integrate(iso_vertices, iso_faces, face_of_patch_mapping);
            surf_int_of_patch.emplace_back(std::move(surf_int));
            vol_int_of_patch.emplace_back(std::move(vol_int));
        }
        timers_manager.pop_timer("compute surface and volume integrals of patches");
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

    // compute order of patches around chains
    // pair<uint32_t, int8_t> : pair (iso-face index, iso-face orientation)
    stl_vector_mp<stl_vector_mp<half_patch_pair_t>> half_patch_pair_list{};
    {
        timers_manager.push_timer("compute order of patches around chains");
        half_patch_pair_list.resize(chains.size());
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
                                half_patch_pair_list[i]);
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
                              arrangement_cells);
            timers_manager.pop_timer("arrangement cells: topo ray shooting");
        }
        timers_manager.pop_timer("compute arrangement cells");
    }

    // fetching function labels (True, False) for each cell
    // stl_vector_mp<bool> sample_function_label(is_positive_scalar_field_sign.begin(),
    //                                           is_positive_scalar_field_sign.begin() + num_funcs + 1);
    // cell_function_labels = sign_propagation(arrangement_cells,
    //                                         shell_of_half_patch,
    //                                         shells,
    //                                         patch_function_labels,
    //                                         num_funcs,
    //                                         sample_function_label);

    return true;
}