#pragma once

#include "primitive_descriptor.h"

typedef struct Blobtree    blobtree_t;     // fixed complete binary tree of 16 layers
typedef struct Node        node_t;         // real node in the tree
typedef struct VirtualNode virtual_node_t; // almost same as node_t, but has parent's and children's pointers to indicate the
                                           // hierarchy, and it is outside of the tree

EXTERN_C_BEGIN

API blobtree_t* create_blobtree();
API void        free_blobtree(blobtree_t* blobtree);

API virtual_node_t* blobtree_new_virtual_node(const constant_descriptor_t* desc);
API void            blobtree_free_virtual_node(virtual_node_t* node);

API void virtual_node_set_parent(virtual_node_t* node, virtual_node_t* parent);
API void virtual_node_set_left_child(virtual_node_t* node, virtual_node_t* child);
API void virtual_node_set_right_child(virtual_node_t* node, virtual_node_t* child);
API void virtual_node_add_child(virtual_node_t* node, virtual_node_t* child);
API void virtual_node_remove_child(virtual_node_t* node, virtual_node_t* child);
API void virtual_node_replace_primitive(virtual_node_t* node, const constant_descriptor_t* desc);

EXTERN_C_END