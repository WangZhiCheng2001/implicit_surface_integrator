#pragma once

#include <tbb/tbb.h>

#include <macros.h>
#include <utils/eigen_alias.hpp>

#include "blobtree.h"

#ifdef _DEBUG
#include <map>
#include <queue>
void output_blobtree(virtual_node_t node);
#endif

BPE_API double evaluate(const constant_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point);
BPE_API double evaluate(const plane_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point);
BPE_API double evaluate(const sphere_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point);
BPE_API double evaluate(const cylinder_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point);
BPE_API double evaluate(const cone_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point);
BPE_API double evaluate(const box_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point);
BPE_API double evaluate(const mesh_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point);
BPE_API double evaluate(const extrude_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point);

BPE_API double evaluate(const primitive_node_t& node, const raw_vector3d_t& point);

// Geometry Generation

/**
 * @brief Create a new empty body
 * @param[in] desc		The empty descriptor
 * @return The virtual node
 */
BPE_API virtual_node_t blobtree_new_virtual_node(const constant_descriptor_t& desc);

/**
 * @brief Create a new plane body
 * @param[in] desc		The plane descriptor
 * @return The virtual node
 */
BPE_API virtual_node_t blobtree_new_virtual_node(const plane_descriptor_t& desc);

/**
 * @brief Create a new sphere body
 * @param[in] desc		The sphere descriptor
 * @return The virtual node
 */
BPE_API virtual_node_t blobtree_new_virtual_node(const sphere_descriptor_t& desc);

/**
 * @brief Create a new cylinder body
 * @param[in] desc		The cylinder descriptor
 * @return The virtual node
 */
BPE_API virtual_node_t blobtree_new_virtual_node(const cylinder_descriptor_t& desc);

/**
 * @brief Create a new cone body
 * @param[in] desc		The cone descriptor
 * @return The virtual node
 */
BPE_API virtual_node_t blobtree_new_virtual_node(const cone_descriptor_t& desc);

/**
 * @brief Create a new box body
 * @param[in] desc		The box descriptor
 * @return The virtual node
 */
BPE_API virtual_node_t blobtree_new_virtual_node(const box_descriptor_t& desc);

/**
 * @brief Create a new mesh body
 * @param[in] desc		The mesh descriptor
 * @return The virtual node
 */
BPE_API virtual_node_t blobtree_new_virtual_node(const mesh_descriptor_t& desc);

/**
 * @brief Create a new extrude body
 * @param[in] desc		The extrude descriptor
 * @return The virtual node
 */
BPE_API virtual_node_t blobtree_new_virtual_node(const extrude_descriptor_t& desc);

/**
 * @brief Free a virtual node
 * @param[in] node		The virtual node to be released
 */
BPE_API void blobtree_free_virtual_node(virtual_node_t* node);

/**
 * @brief Get all primitive nodes
 * @return Primitive array
 */
BPE_API std::vector<primitive_node_t, tbb::tbb_allocator<primitive_node_t>>& get_primitives() noexcept;

// Geometry Operations

/**
 * @brief Union two virtual node, result will be writen to first node
 * @param[in] node1		The first virtual node
 * @param[in] node2		The second virtual node
 */
BPE_API void virtual_node_boolean_union(virtual_node_t* node1, virtual_node_t* node2);

/**
 * @brief Intersect two virtual node, result will be writen to first node
 * @param[in] node1		The first virtual node
 * @param[in] node2		The second virtual node
 */
BPE_API void virtual_node_boolean_intersect(virtual_node_t* node1, virtual_node_t* node2);

/**
 * @brief Difference two virtual node, result will be writen to first node
 * @param[in] node1		The first virtual node
 * @param[in] node2		The second virtual node
 */
BPE_API void virtual_node_boolean_difference(virtual_node_t* node1, virtual_node_t* node2);

/**
 * @brief Offset a body
 * @param[in] node			The virtual node
 * @param[in] directrion	The offset direction
 * @param[in] length		The offset length
 */
BPE_API void virtual_node_offset(virtual_node_t* node, const raw_vector3d_t& direction, const double length);

/**
 * @brief Offset a body
 * @param[in] node		The virtual node
 * @param[in] offset	The offset direction and length
 */
BPE_API void virtual_node_offset(virtual_node_t* node, const raw_vector3d_t& offset);

/**
 * @brief Split a body
 * @param[in] node		The virtual node
 * @param[in] basePoint The base point of the split face
 * @param[in] normal	The normal of the split face
 */
BPE_API void virtual_node_split(virtual_node_t* node, raw_vector3d_t base_point, raw_vector3d_t normal);

// Tree Node Operations

/**
 * @brief Sets the parent of a virtual node, a validity check will be performed
 * @param[in] node		The virtual node whose parent is waiting to be set
 * @param[in] parent	The parent virtual node
 * @return True if the operation is successful
 */
BPE_API bool virtual_node_set_parent(virtual_node_t* node, virtual_node_t* parent);

/**
 * @brief Sets the left node of a virtual node, a validity check will be performed
 * @param[in] node		The virtual node whose left child is waiting to be set
 * @param[in] child		The child virtual node
 * @return True if the operation is successful
 */
BPE_API bool virtual_node_set_left_child(virtual_node_t* node, virtual_node_t* child);

/**
 * @brief Sets the right node of a virtual node, a validity check will be performed
 * @param[in] node		The virtual node whose right child is waiting to be set
 * @param[in] child		The child virtual node
 * @return True if the operation is successful
 */
BPE_API bool virtual_node_set_right_child(virtual_node_t* node, virtual_node_t* child);

/**
 * @brief Add a child node of given virtual node, a validity check will be performed
 * @param[in] node		The virtual node whose child is waiting to be added
 * @param[in] child		The child virtual node
 * @return True if the operation is successful
 */
BPE_API bool virtual_node_add_child(virtual_node_t* node, virtual_node_t* child);

/**
 * @brief Remove a child node of given virtual node, a validity check will be performed
 * @param[in] node		The virtual node whose child is waiting to be removed
 * @param[in] child		The child virtual node
 * @return True if the operation is successful
 */
BPE_API bool virtual_node_remove_child(virtual_node_t* node, virtual_node_t* child);

// Node Replacement Operation

/**
 * @brief Replace a primitive node to constant, a validity check will be performed
 * @param[in] node		The virtual node which points to primitive node
 * @param[in] desc		The new descriptor
 * @return True if the operation is successful
 */
BPE_API bool virtual_node_replace_primitive(virtual_node_t* node, const constant_descriptor_t& desc);

/**
 * @brief Replace a primitive node to plane, a validity check will be performed
 * @param[in] node		The virtual node which points to primitive node
 * @param[in] desc		The new descriptor
 * @return True if the operation is successful
 */
BPE_API bool virtual_node_replace_primitive(virtual_node_t* node, const plane_descriptor_t& desc);

/**
 * @brief Replace a primitive node to sphere, a validity check will be performed
 * @param[in] node		The virtual node which points to primitive node
 * @param[in] desc		The new descriptor
 * @return True if the operation is successful
 */
BPE_API bool virtual_node_replace_primitive(virtual_node_t* node, const sphere_descriptor_t& desc);

/**
 * @brief Replace a primitive node to cylinder, a validity check will be performed
 * @param[in] node		The virtual node which points to primitive node
 * @param[in] desc		The new descriptor
 * @return True if the operation is successful
 */
BPE_API bool virtual_node_replace_primitive(virtual_node_t* node, const cylinder_descriptor_t& desc);

/**
 * @brief Replace a primitive node to cone, a validity check will be performed
 * @param[in] node		The virtual node which points to primitive node
 * @param[in] desc		The new descriptor
 * @return True if the operation is successful
 */
BPE_API bool virtual_node_replace_primitive(virtual_node_t* node, const cone_descriptor_t& desc);

/**
 * @brief Replace a primitive node to box, a validity check will be performed
 * @param[in] node		The virtual node which points to primitive node
 * @param[in] desc		The new descriptor
 * @return True if the operation is successful
 */
BPE_API bool virtual_node_replace_primitive(virtual_node_t* node, const box_descriptor_t& desc);

/**
 * @brief Replace a primitive node to mesh, a validity check will be performed
 * @param[in] node		The virtual node which points to primitive node
 * @param[in] desc		The new descriptor
 * @return True if the operation is successful
 */
BPE_API bool virtual_node_replace_primitive(virtual_node_t* node, const mesh_descriptor_t& desc);

/**
 * @brief Replace a primitive node to extrude, a validity check will be performed
 * @param[in] node		The virtual node which points to primitive node
 * @param[in] desc		The new descriptor
 * @return True if the operation is successful
 */
BPE_API bool virtual_node_replace_primitive(virtual_node_t* node, const extrude_descriptor_t& desc);
