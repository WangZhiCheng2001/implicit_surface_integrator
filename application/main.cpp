#include <iostream>

#include <environment.h>
#include <execution.h>
#include <io.h>

#include <internal_api.hpp>

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
    // auto tree_root = blobtree_new_node(&sphere1, PRIMITIVE_TYPE_SPHERE);
    // auto tree_root = blobtree_new_node(&box, PRIMITIVE_TYPE_BOX);
    auto tree_root = blobtree_new_node(&sphere1, PRIMITIVE_TYPE_SPHERE);
    auto another_sphere_node = blobtree_new_node(&sphere2, PRIMITIVE_TYPE_SPHERE);
    virtual_node_boolean_union(&tree_root, &another_sphere_node);

    std::cout << "Setting environments..." << std::endl;
    setting_descriptor setting_desc{21};
    update_setting(setting_desc);
    update_environment(&tree_root);

    std::cout << "Executing solver..." << std::endl;
    execute_solver(&tree_root);

    std::cout << "Time statistics: " << std::endl;
    print_statistics();

    return 0;
}