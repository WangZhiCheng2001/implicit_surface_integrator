#include "internal_api.hpp"

#include "globals.hpp"
#include "primitive_node_constructor.hpp"
#include "primitive_node_destroyer.hpp"

template <primitive_type type, typename T, typename __desc_constructor, typename __aabb_initer>
bool replace_primitive_node(const virtual_node_t& node,
                            T&&                   desc,
                            __desc_constructor&&  desc_constructor,
                            __aabb_initer&&       aabb_initer)
{
    auto& node_in_tree = structures[node.main_index].nodes[node.inner_index];

    if (!node_fetch_is_primitive(node_in_tree)) { return false; }

    const uint32_t primitive_index = node_fetch_primitive_index(node_in_tree);
    auto&          primitive       = primitives[primitive_index];
    auto&          aabb            = aabbs[primitive_index];

    destroy_primitive_node(primitive);
    primitive = {type, malloc(sizeof(std::remove_cv_t<std::remove_reference_t<T&&>>))};
    aabb.clear();

    aabb_initer(aabb);
    desc_constructor(primitive.desc);

    return true;
}

// ==================================================================================================
// copy replacer
// ==================================================================================================

#define PRIM_NODE_COPY_REPLACER(low_name, high_name, desc_constructor, aabb_initer)                                         \
    BS_API bool virtual_node_replace_primitive(const virtual_node_t& node, const low_name##_descriptor_t& desc)            \
    {                                                                                                                       \
        return replace_primitive_node<PRIMITIVE_TYPE_##high_name>(node,                                                     \
                                                                  desc,                                                     \
                                                                  std::bind(desc_constructor, desc, std::placeholders::_1), \
                                                                  std::bind(aabb_initer, desc, std::placeholders::_1));     \
    }

PRIM_NODE_COPY_REPLACER(constant, CONSTANT, plain_desc_copy_constructor, plain_aabb_initer);
PRIM_NODE_COPY_REPLACER(plane, PLANE, plain_desc_copy_constructor, plain_aabb_initer);
PRIM_NODE_COPY_REPLACER(sphere, SPHERE, plain_desc_copy_constructor, sphere_aabb_initer);
PRIM_NODE_COPY_REPLACER(cylinder, CYLINDER, plain_desc_copy_constructor, cylinder_aabb_initer);
PRIM_NODE_COPY_REPLACER(cone, CONE, plain_desc_copy_constructor, cone_aabb_initer);
PRIM_NODE_COPY_REPLACER(box, BOX, plain_desc_copy_constructor, box_aabb_initer);
PRIM_NODE_COPY_REPLACER(mesh, MESH, mesh_desc_copy_constructor, mesh_aabb_initer);
PRIM_NODE_COPY_REPLACER(extrude, EXTRUDE, extrude_desc_copy_constructor, extrude_aabb_initer);

#undef PRIM_NODE_COPY_REPLACER

// ==================================================================================================
// move replacer
// ==================================================================================================

#define PRIM_NODE_MOVE_REPLACER(low_name, high_name, desc_constructor, aabb_initer)                                         \
    BS_API bool virtual_node_replace_primitive(const virtual_node_t& node, const low_name##_descriptor_t&& desc)           \
    {                                                                                                                       \
        return replace_primitive_node<PRIMITIVE_TYPE_##high_name>(node,                                                     \
                                                                  desc,                                                     \
                                                                  std::bind(desc_constructor, desc, std::placeholders::_1), \
                                                                  std::bind(aabb_initer, desc, std::placeholders::_1));     \
    }

PRIM_NODE_MOVE_REPLACER(constant, CONSTANT, plain_desc_move_constructor, plain_aabb_initer);
PRIM_NODE_MOVE_REPLACER(plane, PLANE, plain_desc_move_constructor, plain_aabb_initer);
PRIM_NODE_MOVE_REPLACER(sphere, SPHERE, plain_desc_move_constructor, sphere_aabb_initer);
PRIM_NODE_MOVE_REPLACER(cylinder, CYLINDER, plain_desc_move_constructor, cylinder_aabb_initer);
PRIM_NODE_MOVE_REPLACER(cone, CONE, plain_desc_move_constructor, cone_aabb_initer);
PRIM_NODE_MOVE_REPLACER(box, BOX, plain_desc_move_constructor, box_aabb_initer);
PRIM_NODE_MOVE_REPLACER(mesh, MESH, mesh_desc_move_constructor, mesh_aabb_initer);
PRIM_NODE_MOVE_REPLACER(extrude, EXTRUDE, extrude_desc_move_constructor, extrude_aabb_initer);

#undef PRIM_NODE_MOVE_REPLACER