#include <cstdlib>

#include "internal_api.hpp"

constexpr auto tree_vector_length = 65535;

void create_new_sub_blobtree(blobtree_t* blobtree)
{
    blobtree->structure_size += 1;
    blobtree->structure       = static_cast<node_t**>(realloc(blobtree->structure, blobtree->structure_size * sizeof(node_t*)));
    if (blobtree->structure == nullptr) { throw std::runtime_error("Memory allocation failed."); }

    blobtree->structure[blobtree->structure_size - 1] = static_cast<node_t*>(malloc(tree_vector_length * sizeof(node_t)));
    if (blobtree->structure[blobtree->structure_size - 1] == nullptr) { throw std::runtime_error("Memory allocation failed."); }
    memset(blobtree->structure[blobtree->structure_size - 1], 0.0, tree_vector_length * sizeof(node_t));
}

void free_sub_blobtree(blobtree_t* blobtree, const int index)
{
    free(blobtree->structure[index]);
    blobtree->structure[index] = nullptr;
}

int get_next_available_index(blobtree_t* blobtree)
{
    for (int i = 0; i < blobtree->structure_size; i++) {
        if (blobtree->structure[i] == nullptr) {
            blobtree->structure[i] = static_cast<node_t*>(malloc(tree_vector_length * sizeof(node_t)));
            if (blobtree->structure[i] == nullptr) { throw std::runtime_error("Memory allocation failed."); }
            memset(blobtree->structure[i], 0.0, tree_vector_length * sizeof(node_t));
            return i;
        }
    }

    create_new_sub_blobtree(blobtree);
    return blobtree->structure_size - 1;
}

blobtree_t* create_blobtree()
{
    blobtree_t* blobtree = static_cast<blobtree_t*>(malloc(sizeof(blobtree_t)));
    if (blobtree == nullptr) { throw std::runtime_error("Memory allocation failed."); }

    blobtree->structure = static_cast<node_t**>(malloc(sizeof(node_t*)));
    if (blobtree->structure == nullptr) { throw std::runtime_error("Memory allocation failed."); }
    blobtree->structure_size = 0;

    blobtree->primitive = static_cast<primitive_node_t*>(malloc(sizeof(primitive_node_t)));
    if (blobtree->primitive == nullptr) { throw std::runtime_error("Memory allocation failed."); }
    blobtree->primitive_size = 0;

    return blobtree;
}

void free_blobtree(blobtree_t* blobtree)
{
    for (int i = 0; i < blobtree->structure_size; i++) { free(blobtree->structure[i]); }
    free(blobtree->structure);
    free(blobtree->primitive);
    free(blobtree);
}

virtual_node_t push_primitive_node(blobtree_t* blobtree, const primitive_node_t& primitive_node)
{
    blobtree->primitive_size += 1;
    blobtree->primitive =
        static_cast<primitive_node_t*>(realloc(blobtree->primitive, (blobtree->primitive_size) * sizeof(primitive_node_t)));
    if (blobtree->primitive == nullptr) { throw std::runtime_error("Memory allocation failed."); }
    blobtree->primitive[blobtree->primitive_size - 1] = primitive_node;

    node_t new_node;
    new_node.non_null   = 1;
    new_node.primitive  = 1;
    new_node.operate    = 3;
    new_node.cross      = 0;
    new_node.index      = blobtree->primitive_size - 1;
    new_node.main_index = 0;

    blobtree->structure[0] = static_cast<node_t*>(realloc(blobtree->structure[0], (blobtree->primitive_size) * sizeof(node_t)));
    if (blobtree->primitive == nullptr) { throw std::runtime_error("Memory allocation failed."); }
    blobtree->structure[0][blobtree->primitive_size - 1] = new_node;

    virtual_node_t virtual_node;
    virtual_node.main_index  = 0;
    virtual_node.inner_index = blobtree->primitive_size - 1;
    return virtual_node;
}

virtual_node_t blobtree_new_virtual_node_constant(const constant_descriptor_t* desc, blobtree_t* blobtree)
{
    primitive_node_t primitive_node;
    primitive_node.desc = (void*)desc;
    primitive_node.type = constant;
    return push_primitive_node(blobtree, primitive_node);
}

virtual_node_t blobtree_new_virtual_node_plane(const plane_descriptor_t* desc, blobtree_t* blobtree)
{
    primitive_node_t primitive_node;
    primitive_node.desc = (void*)desc;
    primitive_node.type = plane;
    return push_primitive_node(blobtree, primitive_node);
}

virtual_node_t blobtree_new_virtual_node_sphere(const sphere_descriptor_t* desc, blobtree_t* blobtree)
{
    primitive_node_t primitive_node;
    primitive_node.desc = (void*)desc;
    primitive_node.type = sphere;
    return push_primitive_node(blobtree, primitive_node);
}

virtual_node_t blobtree_new_virtual_node_cylinder(const cylinder_descriptor_t* desc, blobtree_t* blobtree)
{
    primitive_node_t primitive_node;
    primitive_node.desc = (void*)desc;
    primitive_node.type = cylinder;
    return push_primitive_node(blobtree, primitive_node);
}

virtual_node_t blobtree_new_virtual_node_cone(const cone_descriptor_t* desc, blobtree_t* blobtree)
{
    primitive_node_t primitive_node;
    primitive_node.desc = (void*)desc;
    primitive_node.type = cone;
    return push_primitive_node(blobtree, primitive_node);
}

virtual_node_t blobtree_new_virtual_node_box(const box_descriptor_t* desc, blobtree_t* blobtree)
{
    primitive_node_t primitive_node;
    primitive_node.desc = (void*)desc;
    primitive_node.type = box;
    return push_primitive_node(blobtree, primitive_node);
}

virtual_node_t blobtree_new_virtual_node_mesh(const mesh_descriptor_t* desc, blobtree_t* blobtree)
{
    primitive_node_t primitive_node;
    primitive_node.desc = (void*)desc;
    primitive_node.type = mesh;
    return push_primitive_node(blobtree, primitive_node);
}

virtual_node_t blobtree_new_virtual_node_extrude(const extrude_descriptor_t* desc, blobtree_t* blobtree)
{
    primitive_node_t primitive_node;
    primitive_node.desc = (void*)desc;
    primitive_node.type = extrude;
    return push_primitive_node(blobtree, primitive_node);
}

void blobtree_free_virtual_node(virtual_node_t* node) { ; }

virtual_node_t get_left_child_index(virtual_node_t node, blobtree_t* blobtree)
{
    int  left_child_index = 2 * node.inner_index + 1;
    auto temp             = blobtree->structure[node.main_index][left_child_index];
    if (temp.cross == 2) {
        return virtual_node_t{temp.main_index, temp.index};
    } else if (left_child_index >= tree_vector_length) {
        auto main_index = get_next_available_index(blobtree);
        return virtual_node_t{(unsigned int)main_index, 0};
    } else {
        return virtual_node_t{node.main_index, (unsigned int)left_child_index};
    }
}

virtual_node_t get_right_child_index(virtual_node_t node, blobtree_t* blobtree)
{
    int  right_child_index = 2 * node.inner_index + 2;
    auto temp              = blobtree->structure[node.main_index][right_child_index];
    if (temp.cross == 3) {
        return virtual_node_t{temp.main_index, temp.index};
    } else if (right_child_index >= tree_vector_length) {
        auto main_index = get_next_available_index(blobtree);
        return virtual_node_t{(unsigned int)main_index, 0};
    } else {
        return virtual_node_t{node.main_index, (unsigned int)right_child_index};
    }
}

virtual_node_t get_parent_index(virtual_node_t node, blobtree_t* blobtree)
{
    int  parent_child_index = (node.inner_index - 1) / 2;
    auto temp               = blobtree->structure[node.main_index][parent_child_index];
    if (temp.cross == 1) {
        return virtual_node_t{temp.main_index, temp.index};
    } else {
        return virtual_node_t{node.main_index, (unsigned int)parent_child_index};
    }
}

bool is_primitive_node(node_t node) { return node.primitive == 1; }

bool is_null_node(node_t node) { return node.non_null == 0; }

bool is_left_node(const int index) { return index % 2 == 1; }

bool is_right_node(const int index) { return index % 2 == 0; }

bool is_root_node(const int index) { return index == 0; }

bool update_inner(virtual_node_t old_node, virtual_node_t new_node, blobtree_t* blobtree)
{
    if (is_null_node(blobtree->structure[old_node.main_index][old_node.inner_index])) { return true; }

    if (new_node.inner_index >= tree_vector_length) { return false; }

    if (!is_null_node(blobtree->structure[new_node.main_index][new_node.inner_index])) { return false; }

    if (is_primitive_node(blobtree->structure[new_node.main_index][new_node.inner_index])) {
        blobtree->structure[new_node.main_index][new_node.inner_index] =
            blobtree->structure[old_node.main_index][old_node.inner_index];
        blobtree->structure[old_node.main_index][old_node.inner_index].non_null = 0;
        return true;
    } else {
        if (!update_inner(get_left_child_index(old_node, blobtree), get_left_child_index(new_node, blobtree), blobtree)) {
            return false;
        }
        if (!update_inner(get_right_child_index(old_node, blobtree), get_right_child_index(new_node, blobtree), blobtree)) {
            return false;
        }

        blobtree->structure[new_node.main_index][new_node.inner_index] =
            blobtree->structure[old_node.main_index][old_node.inner_index];
        blobtree->structure[old_node.main_index][old_node.inner_index].non_null = 0;
        return true;
    }
}

void copy_sub_blobtree(const int dst_main_index, const int src_main_index, blobtree_t* blobtree)
{
    memcpy(blobtree->structure[dst_main_index], blobtree->structure[src_main_index], tree_vector_length * sizeof(node_t));
}

bool update(virtual_node_t old_node, virtual_node_t new_node, blobtree_t* blobtree)
{
    // Virtual update, check for out-of-bounds
    auto temp_mian_index = get_next_available_index(blobtree);
    copy_sub_blobtree(temp_mian_index, new_node.main_index, blobtree);

    if (update_inner(old_node, virtual_node_t{(unsigned)temp_mian_index, new_node.inner_index}, blobtree)) {
        copy_sub_blobtree(new_node.main_index, temp_mian_index, blobtree);
        free_sub_blobtree(blobtree, temp_mian_index);
        return true;
    } else {
        free_sub_blobtree(blobtree, temp_mian_index);
        return false;
    }
}

bool virtual_node_boolean_union(virtual_node_t* node1, virtual_node_t* node2, blobtree_t* blobtree) { return false; }

bool virtual_node_boolean_union_save_mode(virtual_node_t* node1, virtual_node_t* node2, blobtree_t* blobtree) { return false; }

bool check(virtual_node_t node, blobtree_t* blobtree)
{
    if (is_null_node(blobtree->structure[node.main_index][node.inner_index])) { return false; }

    if (is_primitive_node(blobtree->structure[node.main_index][node.inner_index])) { return true; }

    if (!check(get_left_child_index(node, blobtree), blobtree)) { return false; }
    if (!check(get_right_child_index(node, blobtree), blobtree)) { return false; }
    return true;
}

bool virtual_node_set_parent(virtual_node_t* node, virtual_node_t* parent, blobtree_t* blobtree)
{
    if (node->main_index == 0) { return false; }

    auto parent_index = get_parent_index(*node, blobtree);
    if (!is_root_node(parent_index.inner_index)
        && !is_null_node(blobtree->structure[parent_index.main_index][parent_index.inner_index])) {
        return false;
    }

    auto left_child_index  = get_left_child_index(*parent, blobtree);
    auto right_child_index = get_right_child_index(*parent, blobtree);
    if (is_left_node(node->inner_index)
        && is_null_node(blobtree->structure[left_child_index.main_index][left_child_index.inner_index])) {
        if (is_root_node(node->inner_index)) {
            if (!update(*node, get_left_child_index(*node, blobtree), blobtree)) { return false; }
        }
        if (!update(*parent, get_parent_index(*node, blobtree), blobtree)) { return false; }
        return true;
    } else if (is_right_node(node->inner_index)
               && is_null_node(blobtree->structure[right_child_index.main_index][right_child_index.inner_index])) {
        if (is_root_node(node->inner_index)) {
            if (!update(*node, get_right_child_index(*node, blobtree), blobtree)) { return false; }
        }
        if (!update(*parent, get_parent_index(*node, blobtree), blobtree)) { return false; }
        return true;
    } else {
        return false;
    }
}

bool virtual_node_set_left_child(virtual_node_t* node, virtual_node_t* child, blobtree_t* blobtree)
{
    int parent_index     = get_parent_index(*child, blobtree).inner_index;
    int left_child_index = get_left_child_index(*node, blobtree).inner_index;

    if (!is_root_node(child->inner_index) && !is_null_node(blobtree->structure[child->main_index][parent_index])) {
        return false;
    }
    if (!is_null_node(blobtree->structure[node->main_index][left_child_index])) { return false; }

    if (update(*child, virtual_node_t{node->main_index, (unsigned int)left_child_index}, blobtree)) {
        *child = *node;
        return true;
    } else {
        blobtree->structure[node->main_index][left_child_index].non_null   = 1;
        blobtree->structure[node->main_index][left_child_index].cross      = 2;
        blobtree->structure[node->main_index][left_child_index].main_index = child->main_index;
        blobtree->structure[node->main_index][left_child_index].index      = child->inner_index;

        blobtree->structure[child->main_index][parent_index].non_null   = 1;
        blobtree->structure[child->main_index][parent_index].cross      = 1;
        blobtree->structure[child->main_index][parent_index].main_index = node->main_index;
        blobtree->structure[child->main_index][parent_index].index      = node->inner_index;
        return true;
    }
}

bool virtual_node_set_right_child(virtual_node_t* node, virtual_node_t* child, blobtree_t* blobtree)
{
    int parent_index      = get_parent_index(*child, blobtree).inner_index;
    int right_child_index = get_right_child_index(*node, blobtree).inner_index;

    if (!is_root_node(child->inner_index) && !is_null_node(blobtree->structure[child->main_index][parent_index])) {
        return false;
    }
    if (!is_null_node(blobtree->structure[node->main_index][right_child_index])) { return false; }

    if (update(*child, virtual_node_t{node->main_index, (unsigned int)right_child_index}, blobtree)) {
        *child = *node;
        return true;
    } else {
        blobtree->structure[node->main_index][right_child_index].non_null   = 1;
        blobtree->structure[node->main_index][right_child_index].cross      = 3;
        blobtree->structure[node->main_index][right_child_index].main_index = child->main_index;
        blobtree->structure[node->main_index][right_child_index].index      = child->inner_index;

        blobtree->structure[child->main_index][parent_index].non_null   = 1;
        blobtree->structure[child->main_index][parent_index].cross      = 1;
        blobtree->structure[child->main_index][parent_index].main_index = node->main_index;
        blobtree->structure[child->main_index][parent_index].index      = node->inner_index;
        return true;
    }
}

bool virtual_node_add_child(virtual_node_t* node, virtual_node_t* child, blobtree_t* blobtree)
{
    auto parent_index = get_parent_index(*child, blobtree).inner_index;
    if (!is_root_node(child->inner_index) && !is_null_node(blobtree->structure[child->main_index][parent_index])) {
        return false;
    }

    if (is_null_node(blobtree->structure[node->main_index][get_left_child_index(*node, blobtree).inner_index])) {
        virtual_node_set_left_child(node, child, blobtree);
        return true;
    } else if (is_null_node(blobtree->structure[node->main_index][get_right_child_index(*node, blobtree).inner_index])) {
        virtual_node_set_right_child(node, child, blobtree);
        return true;
    } else {
        return false;
    }
}

void remove(virtual_node_t node, blobtree_t* blobtree)
{
    if (is_null_node(blobtree->structure[node.main_index][node.inner_index])) { return; }

    blobtree->structure[node.main_index][node.inner_index].non_null = 0;

    if (is_primitive_node(blobtree->structure[node.main_index][node.inner_index])) { return; }

    remove(get_left_child_index(node, blobtree), blobtree);
    remove(get_right_child_index(node, blobtree), blobtree);
}

bool operator==(const virtual_node_t& node1, const virtual_node_t& node2)
{
    return node1.main_index == node2.main_index && node1.inner_index == node2.inner_index;
}

bool virtual_node_remove_child(virtual_node_t* node, virtual_node_t* child, blobtree_t* blobtree)
{
    if (get_left_child_index(*node, blobtree) == *child) {
        remove(*child, blobtree);
        return true;
    } else if (get_right_child_index(*node, blobtree) == *child) {
        remove(*child, blobtree);
        return true;
    } else {
        return false;
    }
}

bool virtual_node_replace_primitive_constant(virtual_node_t* node, const constant_descriptor_t* desc, blobtree_t* blobtree)
{
    if (!is_primitive_node(blobtree->structure[node->main_index][node->inner_index])) { return false; }
    blobtree->primitive[blobtree->structure[node->main_index][node->inner_index].index].desc = (void*)desc;
    blobtree->primitive[blobtree->structure[node->main_index][node->inner_index].index].type = constant;
    return true;
}

bool virtual_node_replace_primitive_plane(virtual_node_t* node, const plane_descriptor_t* desc, blobtree_t* blobtree)
{
    if (!is_primitive_node(blobtree->structure[node->main_index][node->inner_index])) { return false; }
    blobtree->primitive[blobtree->structure[node->main_index][node->inner_index].index].desc = (void*)desc;
    blobtree->primitive[blobtree->structure[node->main_index][node->inner_index].index].type = plane;
    return true;
}

bool virtual_node_replace_primitive_sphere(virtual_node_t* node, const sphere_descriptor_t* desc, blobtree_t* blobtree)
{
    if (!is_primitive_node(blobtree->structure[node->main_index][node->inner_index])) { return false; }
    blobtree->primitive[blobtree->structure[node->main_index][node->inner_index].index].desc = (void*)desc;
    blobtree->primitive[blobtree->structure[node->main_index][node->inner_index].index].type = sphere;
    return true;
}

bool virtual_node_replace_primitive_cylinder(virtual_node_t* node, const cylinder_descriptor_t* desc, blobtree_t* blobtree)
{
    if (!is_primitive_node(blobtree->structure[node->main_index][node->inner_index])) { return false; }
    blobtree->primitive[blobtree->structure[node->main_index][node->inner_index].index].desc = (void*)desc;
    blobtree->primitive[blobtree->structure[node->main_index][node->inner_index].index].type = cylinder;
    return true;
}

bool virtual_node_replace_primitive_cone(virtual_node_t* node, const cone_descriptor_t* desc, blobtree_t* blobtree)
{
    if (!is_primitive_node(blobtree->structure[node->main_index][node->inner_index])) { return false; }
    blobtree->primitive[blobtree->structure[node->main_index][node->inner_index].index].desc = (void*)desc;
    blobtree->primitive[blobtree->structure[node->main_index][node->inner_index].index].type = cone;
    return true;
}

bool virtual_node_replace_primitive_box(virtual_node_t* node, const box_descriptor_t* desc, blobtree_t* blobtree)
{
    if (!is_primitive_node(blobtree->structure[node->main_index][node->inner_index])) { return false; }
    blobtree->primitive[blobtree->structure[node->main_index][node->inner_index].index].desc = (void*)desc;
    blobtree->primitive[blobtree->structure[node->main_index][node->inner_index].index].type = box;
    return true;
}

bool virtual_node_replace_primitive_mesh(virtual_node_t* node, const mesh_descriptor_t* desc, blobtree_t* blobtree)
{
    if (!is_primitive_node(blobtree->structure[node->main_index][node->inner_index])) { return false; }
    blobtree->primitive[blobtree->structure[node->main_index][node->inner_index].index].desc = (void*)desc;
    blobtree->primitive[blobtree->structure[node->main_index][node->inner_index].index].type = mesh;
    return true;
}

bool virtual_node_replace_primitive_extrude(virtual_node_t* node, const extrude_descriptor_t* desc, blobtree_t* blobtree)
{
    if (!is_primitive_node(blobtree->structure[node->main_index][node->inner_index])) { return false; }
    blobtree->primitive[blobtree->structure[node->main_index][node->inner_index].index].desc = (void*)desc;
    blobtree->primitive[blobtree->structure[node->main_index][node->inner_index].index].type = extrude;
    return true;
}
