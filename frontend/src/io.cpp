#include <internal_api.hpp>

#include <io.h>

EXTERN_C_BEGIN

API virtual_node_t blobtree_new_node(const void* desc, primitive_type type)
{
    switch (type) {
        case PRIMITIVE_TYPE_CONSTANT: return blobtree_new_virtual_node(*(const constant_descriptor_t*)desc);
        case PRIMITIVE_TYPE_PLANE:    return blobtree_new_virtual_node(*(const plane_descriptor_t*)desc);
        case PRIMITIVE_TYPE_SPHERE:   return blobtree_new_virtual_node(*(const sphere_descriptor_t*)desc);
        case PRIMITIVE_TYPE_CYLINDER: return blobtree_new_virtual_node(*(const cylinder_descriptor_t*)desc);
        case PRIMITIVE_TYPE_CONE:     return blobtree_new_virtual_node(*(const cone_descriptor_t*)desc);
        case PRIMITIVE_TYPE_BOX:      return blobtree_new_virtual_node(*(const box_descriptor_t*)desc);
        case PRIMITIVE_TYPE_MESH:     return blobtree_new_virtual_node(*(const mesh_descriptor_t*)desc);
        case PRIMITIVE_TYPE_EXTRUDE:  return blobtree_new_virtual_node(*(const extrude_descriptor_t*)desc);
    }
}

EXTERN_C_END