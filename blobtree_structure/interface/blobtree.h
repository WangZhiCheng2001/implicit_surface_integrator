#pragma once

#include <stdint.h>

#include "primitive_descriptor.h"

// double node in the tree
typedef uint32_t node_t;

typedef struct {
    primitive_type type;
    void*          desc; // Type conversion when using
} primitive_node_t;

// almost same as node_t, but has parent's and children's pointers to indicate
// the hierarchy, and it is outside of the tree
typedef struct {
    uint32_t main_index  : 16;
    uint32_t inner_index : 16;
} virtual_node_t;

// fixed complete binary tree of 16 layers
// typedef struct {
//     node_t**          structure;
//     int32_t           structure_size;
//     primitive_node_t* primitive;
//     int32_t           primitive_size;
// } blobtree_t;

// EXTERN_C_BEGIN

// API blobtree_t* create_blobtree();
// API void        free_blobtree(blobtree_t* blobtree);

// API virtual_node_t blobtree_new_virtual_node_constant(const constant_descriptor_t* desc, blobtree_t* blobtree);
// API virtual_node_t blobtree_new_virtual_node_plane(const plane_descriptor_t* desc, blobtree_t* blobtree);
// API virtual_node_t blobtree_new_virtual_node_sphere(const sphere_descriptor_t* desc, blobtree_t* blobtree);
// API virtual_node_t blobtree_new_virtual_node_cylinder(const cylinder_descriptor_t* desc, blobtree_t* blobtree);
// API virtual_node_t blobtree_new_virtual_node_cone(const cone_descriptor_t* desc, blobtree_t* blobtree);
// API virtual_node_t blobtree_new_virtual_node_box(const box_descriptor_t* desc, blobtree_t* blobtree);
// API virtual_node_t blobtree_new_virtual_node_mesh(const mesh_descriptor_t* desc, blobtree_t* blobtree);
// API virtual_node_t blobtree_new_virtual_node_extrude(const extrude_descriptor_t* desc, blobtree_t* blobtree);
// API void           blobtree_free_virtual_node(virtual_node_t* node);

// API bool virtual_node_set_parent(virtual_node_t* node, virtual_node_t* parent, blobtree_t* blobtree);
// API bool virtual_node_set_left_child(virtual_node_t* node, virtual_node_t* child, blobtree_t* blobtree);
// API bool virtual_node_set_right_child(virtual_node_t* node, virtual_node_t* child, blobtree_t* blobtree);
// API bool virtual_node_add_child(virtual_node_t* node, virtual_node_t* child, blobtree_t* blobtree);
// API bool virtual_node_remove_child(virtual_node_t* node, virtual_node_t* child, blobtree_t* blobtree);

// API bool virtual_node_replace_primitive_constant(virtual_node_t* node, const constant_descriptor_t* desc, blobtree_t*
// blobtree); API bool virtual_node_replace_primitive_plane(virtual_node_t* node, const plane_descriptor_t* desc, blobtree_t*
// blobtree); API bool virtual_node_replace_primitive_sphere(virtual_node_t* node, const sphere_descriptor_t* desc, blobtree_t*
// blobtree); API bool virtual_node_replace_primitive_cylinder(virtual_node_t* node, const cylinder_descriptor_t* desc,
// blobtree_t* blobtree); API bool virtual_node_replace_primitive_cone(virtual_node_t* node, const cone_descriptor_t* desc,
// blobtree_t* blobtree); API bool virtual_node_replace_primitive_box(virtual_node_t* node, const box_descriptor_t* desc,
// blobtree_t* blobtree); API bool virtual_node_replace_primitive_mesh(virtual_node_t* node, const mesh_descriptor_t* desc,
// blobtree_t* blobtree); API bool virtual_node_replace_primitive_extrude(virtual_node_t* node, const extrude_descriptor_t*
// desc, blobtree_t* blobtree);

// EXTERN_C_END