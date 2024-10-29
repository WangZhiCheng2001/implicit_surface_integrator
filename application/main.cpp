#include <load_functions.hpp>
#include "implicit_surface_integrator.hpp"

int main()
{
    std::cout << "generating tetrahedron background mesh..." << std::endl;
    raw_point_t aabb_min{-1.0, -1.0, -1.0};
    raw_point_t aabb_max{1.0, 1.0, 1.0};
    auto        background_mesh = generate_tetrahedron_background_mesh(21, aabb_min, aabb_max);
    std::cout << "loading lookup tables..." << std::endl;
    load_lut();
    std::cout << "reading implicit functions..." << std::endl;
    auto implicit_functions = load_functions("functions.json");
    if (!implicit_functions.success) {
        std::cout << "Failed to load implicit functions from file." << std::endl;
        return -1;
    }

    // compute SDF scalar field for the background mesh
    std::cout << "computing SDF scalar field for the background mesh..." << std::endl;
    Eigen::MatrixXd sdf_scalar_field(implicit_functions.functions.size(), background_mesh.vertices.size());
    for (size_t i = 0; i < background_mesh.vertices.size(); ++i) {
        const auto& vertex = background_mesh.vertices[i];
        for (size_t j = 0; j < implicit_functions.functions.size(); ++j) {
            std::visit([&](auto& function) { sdf_scalar_field(j, i) = function.evaluate_scalar(vertex); },
                       implicit_functions.functions[j]);
        }
    }

    ImplicitSurfaceIntegrator integrator(background_mesh, sdf_scalar_field);
    labelled_timers_manager   timers_manager{};
    std::cout << "running implicit surface network integrator..." << std::endl;
    integrator.run(timers_manager);
    integrator.compute();

    return 0;
}