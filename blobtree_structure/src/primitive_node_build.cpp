#include <functional>
#include <iostream>
#include <type_traits>

#include "blobtree.h"
#include "internal_api.hpp"

#include "globals.hpp"
#include "primitive_node_constructor.hpp"

/* Geometry Generation */

virtual_node_t push_primitive_node(primitive_node_t&& primitive_node, const aabb_t&& aabb)
{
    aabbs.emplace_back(aabb);
    primitives.emplace_back(primitive_node);
    counter.emplace_back(1);

    node_t node                      = standard_new_node;
    node_fetch_primitive_index(node) = static_cast<uint32_t>(primitives.size() - 1);

    blobtree_t tree;
    tree.nodes.emplace_back(node);
    tree.leaf_index.push_back(0);

    structures.push_back(tree);

    return virtual_node_t{static_cast<uint32_t>(structures.size() - 1), 0};
}

template <primitive_type type, typename T, typename __desc_constructor, typename __aabb_initer>
virtual_node_t insert_primitive_node(T&& desc, __desc_constructor&& desc_constructor, __aabb_initer&& aabb_initer)
{
    primitive_node_t node{type, malloc(sizeof(std::remove_cv_t<std::remove_reference_t<T>>))};
    aabb_t           aabb{};
    aabb_initer(aabb);
    desc_constructor(node.desc);
    return push_primitive_node(std::move(node), std::move(aabb));
}

BS_API void blobtree_free_virtual_node(const virtual_node_t& node) { free_sub_blobtree(node.main_index); }

// ==================================================================================================
// copy constrctor
// ==================================================================================================

#define PRIM_NODE_COPY_CONSTRUCTOR(low_name, high_name, desc_constructor, aabb_initer)                                     \
    BS_API virtual_node_t blobtree_new_virtual_node(const low_name##_descriptor_t& desc)                                   \
    {                                                                                                                      \
        return insert_primitive_node<PRIMITIVE_TYPE_##high_name>(desc,                                                     \
                                                                 std::bind(desc_constructor, desc, std::placeholders::_1), \
                                                                 std::bind(aabb_initer, desc, std::placeholders::_1));     \
    }

PRIM_NODE_COPY_CONSTRUCTOR(constant, CONSTANT, plain_desc_copy_constructor, plain_aabb_initer);
PRIM_NODE_COPY_CONSTRUCTOR(plane, PLANE, plain_desc_copy_constructor, plain_aabb_initer);
PRIM_NODE_COPY_CONSTRUCTOR(sphere, SPHERE, plain_desc_copy_constructor, sphere_aabb_initer);
PRIM_NODE_COPY_CONSTRUCTOR(cylinder, CYLINDER, plain_desc_copy_constructor, cylinder_aabb_initer);
PRIM_NODE_COPY_CONSTRUCTOR(cone, CONE, plain_desc_copy_constructor, cone_aabb_initer);
PRIM_NODE_COPY_CONSTRUCTOR(box, BOX, plain_desc_copy_constructor, box_aabb_initer);
PRIM_NODE_COPY_CONSTRUCTOR(mesh, MESH, mesh_desc_copy_constructor, mesh_aabb_initer);
PRIM_NODE_COPY_CONSTRUCTOR(extrude, EXTRUDE, extrude_desc_copy_constructor, extrude_aabb_initer);

#undef PRIM_NODE_COPY_CONSTRUCTOR

// ==================================================================================================
// move constrctor
// ==================================================================================================

#define PRIM_NODE_MOVE_CONSTRUCTOR(low_name, high_name, desc_constructor, aabb_initer)                                     \
    BS_API virtual_node_t blobtree_new_virtual_node(const low_name##_descriptor_t&& desc)                                  \
    {                                                                                                                      \
        return insert_primitive_node<PRIMITIVE_TYPE_##high_name>(desc,                                                     \
                                                                 std::bind(desc_constructor, desc, std::placeholders::_1), \
                                                                 std::bind(aabb_initer, desc, std::placeholders::_1));     \
    }

PRIM_NODE_MOVE_CONSTRUCTOR(constant, CONSTANT, plain_desc_move_constructor, plain_aabb_initer);
PRIM_NODE_MOVE_CONSTRUCTOR(plane, PLANE, plain_desc_move_constructor, plain_aabb_initer);
PRIM_NODE_MOVE_CONSTRUCTOR(sphere, SPHERE, plain_desc_move_constructor, sphere_aabb_initer);
PRIM_NODE_MOVE_CONSTRUCTOR(cylinder, CYLINDER, plain_desc_move_constructor, cylinder_aabb_initer);
PRIM_NODE_MOVE_CONSTRUCTOR(cone, CONE, plain_desc_move_constructor, cone_aabb_initer);
PRIM_NODE_MOVE_CONSTRUCTOR(box, BOX, plain_desc_move_constructor, box_aabb_initer);
PRIM_NODE_MOVE_CONSTRUCTOR(mesh, MESH, mesh_desc_move_constructor, mesh_aabb_initer);
PRIM_NODE_MOVE_CONSTRUCTOR(extrude, EXTRUDE, extrude_desc_move_constructor, extrude_aabb_initer);

#undef PRIM_NODE_MOVE_CONSTRUCTOR