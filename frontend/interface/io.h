#pragma once

#include <blobtree.h>
#include <macros.h>

EXTERN_C_BEGIN

API virtual_node_t blobtree_new_node(const void* desc, primitive_type type);

EXTERN_C_END