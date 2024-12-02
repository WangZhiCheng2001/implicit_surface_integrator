#include "internal_api.hpp"

#include "globals.hpp"

/* Geometry Generation */

virtual_node_t push_primitive_node(primitive_node_t&& primitive_node, const aabb_t&& aabb)
{
    aabbs.emplace_back(aabb);
    primitives.emplace_back(primitive_node);

    node_t node                      = standard_new_node;
    node_fetch_primitive_index(node) = static_cast<uint32_t>(primitives.size() - 1);

    blobtree_t tree;
    tree.nodes.emplace_back(node);
    tree.leaf_index.push_back(0);

    structures.push_back(tree);

    return virtual_node_t{static_cast<uint32_t>(structures.size() - 1), 0};
}

BPE_API virtual_node_t blobtree_new_virtual_node(const constant_descriptor_t& desc)
{
    primitive_node_t node{PRIMITIVE_TYPE_CONSTANT, malloc(sizeof(constant_descriptor_t))};
    *((constant_descriptor_t*)node.desc) = std::move(desc);

    aabb_t aabb{};
    return push_primitive_node(std::move(node), std::move(aabb));
}

BPE_API virtual_node_t blobtree_new_virtual_node(const plane_descriptor_t& desc)
{
    primitive_node_t node{PRIMITIVE_TYPE_PLANE, malloc(sizeof(plane_descriptor_t))};
    *((plane_descriptor_t*)node.desc) = std::move(desc);

    aabb_t aabb{};
    return push_primitive_node(std::move(node), std::move(aabb));
}

BPE_API virtual_node_t blobtree_new_virtual_node(const sphere_descriptor_t& desc)
{
    primitive_node_t node{PRIMITIVE_TYPE_SPHERE, malloc(sizeof(sphere_descriptor_t))};

    Eigen::Map<const Eigen::Vector3d> center(&desc.center.x);

    aabb_t aabb{center.array() - desc.radius, center.array() + desc.radius};

    *((sphere_descriptor_t*)node.desc) = std::move(desc);
    return push_primitive_node(std::move(node), std::move(aabb));
}

BPE_API virtual_node_t blobtree_new_virtual_node(const cylinder_descriptor_t& desc)
{
    primitive_node_t node{PRIMITIVE_TYPE_CYLINDER, malloc(sizeof(cylinder_descriptor_t))};

    // NOTE: A rough AABB bounding box
    aabb_t                            aabb{};
    Eigen::Map<const Eigen::Vector3d> bottom_center(&desc.bottom_origion.x), offset(&desc.offset.x);
    aabb.extend(bottom_center.array() + desc.radius);
    aabb.extend(bottom_center.array() - desc.radius);
    aabb.extend(bottom_center.array() + offset.array() + desc.radius);
    aabb.extend(bottom_center.array() + offset.array() - desc.radius);

    *((cylinder_descriptor_t*)node.desc) = std::move(desc);
    return push_primitive_node(std::move(node), std::move(aabb));
}

BPE_API virtual_node_t blobtree_new_virtual_node(const cone_descriptor_t& desc)
{
    primitive_node_t node{PRIMITIVE_TYPE_CONE, malloc(sizeof(cone_descriptor_t))};

    // NOTE: A rough AABB bounding box
    aabb_t                            aabb{};
    Eigen::Map<const Eigen::Vector3d> top_point(&desc.top_point.x), bottom_point(&desc.bottom_point.x);
    aabb.extend(top_point.array() + desc.radius1);
    aabb.extend(top_point.array() - desc.radius1);
    aabb.extend(bottom_point.array() + desc.radius2);
    aabb.extend(bottom_point.array() - desc.radius2);

    *((cone_descriptor_t*)node.desc) = std::move(desc);
    return push_primitive_node(std::move(node), std::move(aabb));
}

BPE_API virtual_node_t blobtree_new_virtual_node(const box_descriptor_t& desc)
{
    primitive_node_t node{PRIMITIVE_TYPE_BOX, malloc(sizeof(box_descriptor_t))};

    Eigen::Map<const Eigen::Vector3d> center(&desc.center.x), half_size(&desc.half_size.x);

    aabb_t aabb{center - half_size, center + half_size};

    *((box_descriptor_t*)node.desc) = std::move(desc);
    return push_primitive_node(std::move(node), std::move(aabb));
}

BPE_API virtual_node_t blobtree_new_virtual_node(const mesh_descriptor_t& desc)
{
    primitive_node_t node{PRIMITIVE_TYPE_MESH, malloc(sizeof(mesh_descriptor_t))};

    aabb_t aabb{};
    for (int i = 0; i < desc.point_number; i++) { aabb.extend(Eigen::Map<const Eigen::Vector3d>(&desc.points[i].x)); }

    *((mesh_descriptor_t*)node.desc) = std::move(desc);
    return push_primitive_node(std::move(node), std::move(aabb));
}

BPE_API virtual_node_t blobtree_new_virtual_node(const extrude_descriptor_t& desc)
{
    primitive_node_t node{PRIMITIVE_TYPE_EXTRUDE, malloc(sizeof(extrude_descriptor_t))};

    aabb_t                            aabb{};
    Eigen::Map<const Eigen::Vector3d> e(&desc.extusion.x);
    // NOTE: Currently only straight edges are considered
    for (int i = 0; i < desc.edges_number; i++) {
        Eigen::Map<const Eigen::Vector3d> p(&desc.points[i].x);
        aabb.extend(p);
        aabb.extend(p + e);
    }

    *((extrude_descriptor_t*)node.desc) = std::move(desc);
    return push_primitive_node(std::move(node), std::move(aabb));
}

BPE_API void blobtree_free_virtual_node(const virtual_node_t& node) { free_sub_blobtree(node.main_index); }
