#pragma once

#include <stdbool.h>

#include <blobtree.h>
#include <macros.h>

typedef struct {
    void* desc;
} copyable_descriptor_t;

typedef struct {
    void* desc;
} movable_descriptor_t;

EXTERN_C_BEGIN

/**
 * @brief Create a new primitive body
 * @param[in] desc		The descriptor of the primitive, must be consistent with the type
 * @param[in] type		The type of the primitive
 * @return The virtual node (indices) pointing to the created primitive node
 */
API virtual_node_t blobtree_new_node_by_copy(const copyable_descriptor_t desc, primitive_type type);
API virtual_node_t blobtree_new_node_by_move(const movable_descriptor_t desc, primitive_type type);

/**
 * @brief Union two virtual node, result will be writen to first node
 * @param[in] node1		The first virtual node
 * @param[in] node2		The second virtual node
 */
API void virtual_node_boolean_union(virtual_node_t* node1, const virtual_node_t* node2);

/**
 * @brief Intersect two virtual node, result will be writen to first node
 * @param[in] node1		The first virtual node
 * @param[in] node2		The second virtual node
 */
API void virtual_node_boolean_intersect(virtual_node_t* node1, const virtual_node_t* node2);

/**
 * @brief Difference two virtual node, result will be writen to first node
 * @param[in] node1		The first virtual node
 * @param[in] node2		The second virtual node
 */
API void virtual_node_boolean_difference(virtual_node_t* node1, const virtual_node_t* node2);

/**
 * @brief Offset a body
 * @param[in] node			The virtual node
 * @param[in] directrion	The offset direction
 * @param[in] length		The offset length
 */
API void virtual_node_offset(virtual_node_t* node, const raw_vector3d_t& direction, const double length);

/**
 * @brief Offset a body
 * @param[in] node		The virtual node
 * @param[in] offset	The offset direction and length
 */
API void virtual_node_offset_directly(virtual_node_t* node, const raw_vector3d_t& offset);

/**
 * @brief Split a body
 * @param[in] node		The virtual node
 * @param[in] basePoint The base point of the split face
 * @param[in] normal	The normal of the split face
 */
API void virtual_node_split(virtual_node_t* node, raw_vector3d_t base_point, raw_vector3d_t normal);

// Tree Node Operations

/**
 * @brief Sets the parent of a virtual node, a validity check will be performed
 * @param[in] node		The virtual node whose parent is waiting to be set
 * @param[in] parent	The parent virtual node
 * @return True if the operation is successful
 */
API bool virtual_node_set_parent(const virtual_node_t* node, const virtual_node_t* parent);

/**
 * @brief Sets the left node of a virtual node, a validity check will be performed
 * @param[in] node		The virtual node whose left child is waiting to be set
 * @param[in] child		The child virtual node
 * @return True if the operation is successful
 */
API bool virtual_node_set_left_child(const virtual_node_t* node, const virtual_node_t* child);

/**
 * @brief Sets the right node of a virtual node, a validity check will be performed
 * @param[in] node		The virtual node whose right child is waiting to be set
 * @param[in] child		The child virtual node
 * @return True if the operation is successful
 */
API bool virtual_node_set_right_child(const virtual_node_t* node, const virtual_node_t* child);

/**
 * @brief Add a child node of given virtual node, a validity check will be performed
 * @param[in] node		The virtual node whose child is waiting to be added
 * @param[in] child		The child virtual node
 * @return True if the operation is successful
 */
API bool virtual_node_add_child(const virtual_node_t* node, const virtual_node_t* child);

/**
 * @brief Remove a child node of given virtual node, a validity check will be performed
 * @param[in] node		The virtual node whose child is waiting to be removed
 * @param[in] child		The child virtual node
 * @return True if the operation is successful
 */
API bool virtual_node_remove_child(const virtual_node_t* node, const virtual_node_t* child);

/**
 * @brief Replace a primitive node to a new one, a validity check will be performed
 * @param[in] node		The virtual node which points to primitive node
 * @param[in] desc		The new descriptor
 * @return True if the operation is successful
 */
API bool virtual_node_replace_primitive_by_copy(const virtual_node_t*       node,
                                                const copyable_descriptor_t desc,
                                                primitive_type              type);
API bool virtual_node_replace_primitive_by_move(const virtual_node_t*      node,
                                                const movable_descriptor_t desc,
                                                primitive_type             type);

EXTERN_C_END