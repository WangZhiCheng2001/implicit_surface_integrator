#include <iostream>
#include <array>

#include <environment.h>
#include <execution.h>
#include <io.h>

#include <construct_helper.hpp>
#include "primitive_descriptor.h"

int main()
{
    std::cout << "Setting scene..." << std::endl;
    sphere_descriptor_t sphere1{
        {.0, .0, .0},
        0.5
    };
    sphere_descriptor_t sphere2{
        {.01, .0, .0},
        0.5
    };
    box_descriptor_t box{
        {0., 0., 0.},
        {1., 1., 1.}
    };
    auto points = std::array{
        raw_vector3d_t{-7200.0000000000282, -7479.9999999993715, 0.0},
        raw_vector3d_t{-4420.0000000000000, -7479.9999999993724, 0.0},
        raw_vector3d_t{-4420.0000000000000, -7719.9999999993724, 0.0},
        raw_vector3d_t{-7200.0000000000282, -7719.9999999993715, 0.0}
    };
    auto                 buldges = std::array{0.0, 0.0, 0.0, 0.0};
    extrude_descriptor_t extrude{
        static_cast<uint32_t>(buldges.size()),
        raw_vector3d_t{0.0, 0.0, 78.000000000251021},
        points.data(),
        buldges.data()
    };
    // auto tree_root = blobtree_new_node(&sphere1, PRIMITIVE_TYPE_SPHERE);
    // auto tree_root = blobtree_new_node(&box, PRIMITIVE_TYPE_BOX);
    // auto tree_root = blobtree_new_node(&sphere1, PRIMITIVE_TYPE_SPHERE);
    // auto another_sphere_node = blobtree_new_node(&sphere2, PRIMITIVE_TYPE_SPHERE);
    // virtual_node_boolean_union(&tree_root, &another_sphere_node);
    auto tree_root = make_primitive_node_by_move(box);
    // auto tree_root = make_primitive_node_by_move(extrude);

    std::cout << "Setting environments..." << std::endl;
    setting_descriptor setting_desc{21, 1e-5};
    update_setting(setting_desc);
    update_environment(&tree_root);

    std::cout << "Executing solver..." << std::endl;
    auto result = execute_solver(&tree_root);
    std::cout << "Surface integral result: " << result.surf_int_result << std::endl;
    std::cout << "Volume integral result: " << result.vol_int_result << std::endl;

    std::cout << "Time statistics: " << std::endl;
    print_statistics();

    return 0;
}