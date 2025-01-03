#pragma once

#include <tbb/tbb.h>

#include <macros.h>
#include <utils/eigen_alias.hpp>

#include "blobtree.h"
#include "internal_structs.hpp"

// ======================================================================================================
// Debugging
// ======================================================================================================

#ifdef _DEBUG
#include <map>
#include <queue>
void output_blobtree(virtual_node_t node);
#endif

// ======================================================================================================
// APIs
// ======================================================================================================

// Basic Operations

BS_API size_t blobtree_get_node_count(uint32_t index) noexcept;
BS_API std::vector<uint32_t, tbb::tbb_allocator<uint32_t>> blobtree_get_leaf_nodes(uint32_t index) noexcept;
BS_API node_t&                                             blobtree_get_node(const virtual_node_t& node) noexcept;

BS_API size_t                  get_primitive_count() noexcept;
BS_API const primitive_node_t& get_primitive_node(uint32_t index) noexcept;
BS_API const aabb_t&           get_aabb(uint32_t index) noexcept;

BS_API void free_sub_blobtree(uint32_t index) noexcept;

BS_API void clear_blobtree() noexcept;

// Geometry Generation

BS_API virtual_node_t blobtree_new_virtual_node(const constant_descriptor_t& desc);
BS_API virtual_node_t blobtree_new_virtual_node(const plane_descriptor_t& desc);
BS_API virtual_node_t blobtree_new_virtual_node(const sphere_descriptor_t& desc);
BS_API virtual_node_t blobtree_new_virtual_node(const cylinder_descriptor_t& desc);
BS_API virtual_node_t blobtree_new_virtual_node(const cone_descriptor_t& desc);
BS_API virtual_node_t blobtree_new_virtual_node(const box_descriptor_t& desc);
BS_API virtual_node_t blobtree_new_virtual_node(const mesh_descriptor_t& desc);
BS_API virtual_node_t blobtree_new_virtual_node(const extrude_polyline_descriptor_t& desc);
BS_API virtual_node_t blobtree_new_virtual_node(const extrude_arcline_descriptor_t& desc);
BS_API virtual_node_t blobtree_new_virtual_node(const extrude_helixline_descriptor_t& desc);

BS_API virtual_node_t blobtree_new_virtual_node(const constant_descriptor_t&& desc);
BS_API virtual_node_t blobtree_new_virtual_node(const plane_descriptor_t&& desc);
BS_API virtual_node_t blobtree_new_virtual_node(const sphere_descriptor_t&& desc);
BS_API virtual_node_t blobtree_new_virtual_node(const cylinder_descriptor_t&& desc);
BS_API virtual_node_t blobtree_new_virtual_node(const cone_descriptor_t&& desc);
BS_API virtual_node_t blobtree_new_virtual_node(const box_descriptor_t&& desc);
BS_API virtual_node_t blobtree_new_virtual_node(const mesh_descriptor_t&& desc);
BS_API virtual_node_t blobtree_new_virtual_node(const extrude_polyline_descriptor_t&& desc);
BS_API virtual_node_t blobtree_new_virtual_node(const extrude_arcline_descriptor_t&& desc);
BS_API virtual_node_t blobtree_new_virtual_node(const extrude_helixline_descriptor_t&& desc);

BS_API void blobtree_free_virtual_node(const virtual_node_t& node);

// Geometry Operations

BS_API void virtual_node_boolean_union(virtual_node_t& node1, const virtual_node_t& node2);
BS_API void virtual_node_boolean_intersect(virtual_node_t& node1, const virtual_node_t& node2);
BS_API void virtual_node_boolean_difference(virtual_node_t& node1, const virtual_node_t& node2);
BS_API void virtual_node_offset(virtual_node_t& node, const raw_vector3d_t& direction, const double length);
BS_API void virtual_node_offset(virtual_node_t& node, const raw_vector3d_t& offset);
BS_API void virtual_node_split(virtual_node_t& node, raw_vector3d_t base_point, raw_vector3d_t normal);

// Tree Node Operations

BS_API bool virtual_node_set_parent(const virtual_node_t& node, const virtual_node_t& parent);
BS_API bool virtual_node_set_left_child(const virtual_node_t& node, const virtual_node_t& child);
BS_API bool virtual_node_set_right_child(const virtual_node_t& node, const virtual_node_t& child);
BS_API bool virtual_node_add_child(const virtual_node_t& node, const virtual_node_t& child);
BS_API bool virtual_node_remove_child(const virtual_node_t& node, const virtual_node_t& child);

// Node Replacement Operation

BS_API bool virtual_node_replace_primitive(const virtual_node_t& node, const constant_descriptor_t& desc);
BS_API bool virtual_node_replace_primitive(const virtual_node_t& node, const plane_descriptor_t& desc);
BS_API bool virtual_node_replace_primitive(const virtual_node_t& node, const sphere_descriptor_t& desc);
BS_API bool virtual_node_replace_primitive(const virtual_node_t& node, const cylinder_descriptor_t& desc);
BS_API bool virtual_node_replace_primitive(const virtual_node_t& node, const cone_descriptor_t& desc);
BS_API bool virtual_node_replace_primitive(const virtual_node_t& node, const box_descriptor_t& desc);
BS_API bool virtual_node_replace_primitive(const virtual_node_t& node, const mesh_descriptor_t& desc);
BS_API bool virtual_node_replace_primitive(const virtual_node_t& node, const extrude_polyline_descriptor_t& desc);
BS_API bool virtual_node_replace_primitive(const virtual_node_t& node, const extrude_arcline_descriptor_t& desc);
BS_API bool virtual_node_replace_primitive(const virtual_node_t& node, const extrude_helixline_descriptor_t& desc);

BS_API bool virtual_node_replace_primitive(const virtual_node_t& node, const constant_descriptor_t&& desc);
BS_API bool virtual_node_replace_primitive(const virtual_node_t& node, const plane_descriptor_t&& desc);
BS_API bool virtual_node_replace_primitive(const virtual_node_t& node, const sphere_descriptor_t&& desc);
BS_API bool virtual_node_replace_primitive(const virtual_node_t& node, const cylinder_descriptor_t&& desc);
BS_API bool virtual_node_replace_primitive(const virtual_node_t& node, const cone_descriptor_t&& desc);
BS_API bool virtual_node_replace_primitive(const virtual_node_t& node, const box_descriptor_t&& desc);
BS_API bool virtual_node_replace_primitive(const virtual_node_t& node, const mesh_descriptor_t&& desc);
BS_API bool virtual_node_replace_primitive(const virtual_node_t& node, const extrude_polyline_descriptor_t&& desc);
BS_API bool virtual_node_replace_primitive(const virtual_node_t& node, const extrude_arcline_descriptor_t&& desc);
BS_API bool virtual_node_replace_primitive(const virtual_node_t& node, const extrude_helixline_descriptor_t&& desc);
