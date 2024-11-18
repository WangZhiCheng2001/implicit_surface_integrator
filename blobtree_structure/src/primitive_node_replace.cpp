#include "internal_api.hpp"

#include "globals.hpp"
#include "aabb.hpp"
#include "node_operation.hpp"

BPE_API bool virtual_node_replace_primitive(virtual_node_t* node, const constant_descriptor_t& desc)
{
    auto& node_in_tree = structures[node->main_index].nodes[node->inner_index];

    if (!node_fetch_is_primitive(node_in_tree)) { return false; }

    const uint32_t primitive_index                              = node_fetch_primitive_index(node_in_tree);
    *((constant_descriptor_t*)primitives[primitive_index].desc) = std::move(desc);
    primitives[primitive_index].type                            = PRIMITIVE_TYPE_CONSTANT;
    aabbs[primitive_index]                                      = aabb_t{};

    return true;
}

BPE_API bool virtual_node_replace_primitive(virtual_node_t* node, const plane_descriptor_t& desc)
{
    auto& node_in_tree = structures[node->main_index].nodes[node->inner_index];

    if (!node_fetch_is_primitive(node_in_tree)) { return false; }

    const uint32_t primitive_index                           = node_fetch_primitive_index(node_in_tree);
    *((plane_descriptor_t*)primitives[primitive_index].desc) = std::move(desc);
    primitives[primitive_index].type                         = PRIMITIVE_TYPE_PLANE;
    aabbs[primitive_index]                                   = aabb_t{};

    return true;
}

BPE_API bool virtual_node_replace_primitive(virtual_node_t* node, const sphere_descriptor_t& desc)
{
    auto& node_in_tree = structures[node->main_index].nodes[node->inner_index];

    if (!node_fetch_is_primitive(node_in_tree)) { return false; }

    const uint32_t primitive_index = node_fetch_primitive_index(node_in_tree);

    Eigen::Map<const Eigen::Vector3d> center(&desc.center.x);

    aabb_t aabb{center.array() - desc.radius, center.array() + desc.radius};

    *((sphere_descriptor_t*)primitives[primitive_index].desc) = std::move(desc);
    primitives[primitive_index].type                          = PRIMITIVE_TYPE_SPHERE;
    aabbs[primitive_index]                                    = std::move(aabb);

    return true;
}

BPE_API bool virtual_node_replace_primitive(virtual_node_t* node, const cylinder_descriptor_t& desc)
{
    auto& node_in_tree = structures[node->main_index].nodes[node->inner_index];

    if (!node_fetch_is_primitive(node_in_tree)) { return false; }

    const uint32_t primitive_index = node_fetch_primitive_index(node_in_tree);

    // NOTE: A rough AABB bounding box
    aabb_t                            aabb{};
    Eigen::Map<const Eigen::Vector3d> bottom_center(&desc.bottom_origion.x), offset(&desc.offset.x);
    aabb.extend(bottom_center.array() + desc.radius);
    aabb.extend(bottom_center.array() - desc.radius);
    aabb.extend(bottom_center.array() + offset.array() + desc.radius);
    aabb.extend(bottom_center.array() + offset.array() - desc.radius);

    *((cylinder_descriptor_t*)primitives[primitive_index].desc) = std::move(desc);
    primitives[primitive_index].type                            = PRIMITIVE_TYPE_CYLINDER;
    aabbs[primitive_index]                                      = std::move(aabb);

    return true;
}

BPE_API bool virtual_node_replace_primitive(virtual_node_t* node, const cone_descriptor_t& desc)
{
    auto& node_in_tree = structures[node->main_index].nodes[node->inner_index];

    if (!node_fetch_is_primitive(node_in_tree)) { return false; }

    const uint32_t primitive_index = node_fetch_primitive_index(node_in_tree);

    // NOTE: A rough AABB bounding box
    aabb_t                            aabb{};
    Eigen::Map<const Eigen::Vector3d> top_point(&desc.top_point.x), bottom_point(&desc.bottom_point.x);
    aabb.extend(top_point.array() + desc.radius1);
    aabb.extend(top_point.array() - desc.radius1);
    aabb.extend(bottom_point.array() + desc.radius2);
    aabb.extend(bottom_point.array() - desc.radius2);

    *((cone_descriptor_t*)primitives[primitive_index].desc) = std::move(desc);
    primitives[primitive_index].type                        = PRIMITIVE_TYPE_CONE;
    aabbs[primitive_index]                                  = std::move(aabb);

    return true;
}

BPE_API bool virtual_node_replace_primitive(virtual_node_t* node, const box_descriptor_t& desc)
{
    auto& node_in_tree = structures[node->main_index].nodes[node->inner_index];

    if (!node_fetch_is_primitive(node_in_tree)) { return false; }

    const uint32_t primitive_index = node_fetch_primitive_index(node_in_tree);

    Eigen::Map<const Eigen::Vector3d> center(&desc.center.x), half_size(&desc.half_size.x);

    aabb_t aabb{center - half_size, center + half_size};

    *((box_descriptor_t*)primitives[primitive_index].desc) = std::move(desc);
    primitives[primitive_index].type                       = PRIMITIVE_TYPE_BOX;
    aabbs[primitive_index]                                 = std::move(aabb);

    return true;
}

BPE_API bool virtual_node_replace_primitive(virtual_node_t* node, const mesh_descriptor_t& desc)
{
    auto& node_in_tree = structures[node->main_index].nodes[node->inner_index];

    if (!node_fetch_is_primitive(node_in_tree)) { return false; }

    const uint32_t primitive_index = node_fetch_primitive_index(node_in_tree);

    aabb_t aabb{};
    for (int i = 0; i < desc.point_number; i++) { aabb.extend(Eigen::Map<const Eigen::Vector3d>(&desc.points[i].x)); }

    *((mesh_descriptor_t*)primitives[primitive_index].desc) = std::move(desc);
    primitives[primitive_index].type                        = PRIMITIVE_TYPE_MESH;
    aabbs[primitive_index]                                  = std::move(aabb);

    return true;
}

BPE_API bool virtual_node_replace_primitive(virtual_node_t* node, const extrude_descriptor_t& desc)
{
    auto& node_in_tree = structures[node->main_index].nodes[node->inner_index];

    if (!node_fetch_is_primitive(node_in_tree)) { return false; }

    const uint32_t primitive_index = node_fetch_primitive_index(node_in_tree);

    aabb_t                            aabb{};
    Eigen::Map<const Eigen::Vector3d> e(&desc.extusion.x);
    // NOTE: Currently only straight edges are considered
    for (int i = 0; i < desc.edges_number; i++) {
        Eigen::Map<const Eigen::Vector3d> p(&desc.points[i].x);
        aabb.extend(p);
        aabb.extend(p + e);
    }

    *((extrude_descriptor_t*)primitives[primitive_index].desc) = std::move(desc);
    primitives[primitive_index].type                           = PRIMITIVE_TYPE_EXTRUDE;
    aabbs[primitive_index]                                     = std::move(aabb);

    return true;
}