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

API void virtual_node_boolean_union(virtual_node_t* node1, const virtual_node_t* node2)
{
    virtual_node_boolean_union(*node1, *node2);
}

API void virtual_node_boolean_intersect(virtual_node_t* node1, const virtual_node_t* node2)
{
    virtual_node_boolean_intersect(*node1, *node2);
}

API void virtual_node_boolean_difference(virtual_node_t* node1, const virtual_node_t* node2)
{
    virtual_node_boolean_difference(*node1, *node2);
}

API void virtual_node_offset(virtual_node_t* node, const raw_vector3d_t& direction, const double length)
{
    virtual_node_offset(*node, direction, length);
}

API void virtual_node_offset_directly(virtual_node_t* node, const raw_vector3d_t& offset)
{
    virtual_node_offset(*node, offset);
}

API void virtual_node_split(virtual_node_t* node, raw_vector3d_t base_point, raw_vector3d_t normal)
{
    virtual_node_split(*node, base_point, normal);
}

API bool virtual_node_set_parent(const virtual_node_t* node, const virtual_node_t* parent)
{
    return virtual_node_set_parent(*node, *parent);
}

API bool virtual_node_set_left_child(const virtual_node_t* node, const virtual_node_t* child)
{
    return virtual_node_set_left_child(*node, *child);
}

API bool virtual_node_set_right_child(const virtual_node_t* node, const virtual_node_t* child)
{
    return virtual_node_set_right_child(*node, *child);
}

API bool virtual_node_add_child(const virtual_node_t* node, const virtual_node_t* child)
{
    return virtual_node_add_child(*node, *child);
}

API bool virtual_node_remove_child(const virtual_node_t* node, const virtual_node_t* child)
{
    return virtual_node_remove_child(*node, *child);
}

API bool virtual_node_replace_primitive(const virtual_node_t* node, const void* desc, primitive_type type)
{
    switch (type) {
        case PRIMITIVE_TYPE_CONSTANT: return virtual_node_replace_primitive(*node, *(const constant_descriptor_t*)desc);
        case PRIMITIVE_TYPE_PLANE:    return virtual_node_replace_primitive(*node, *(const plane_descriptor_t*)desc);
        case PRIMITIVE_TYPE_SPHERE:   return virtual_node_replace_primitive(*node, *(const sphere_descriptor_t*)desc);
        case PRIMITIVE_TYPE_CYLINDER: return virtual_node_replace_primitive(*node, *(const cylinder_descriptor_t*)desc);
        case PRIMITIVE_TYPE_CONE:     return virtual_node_replace_primitive(*node, *(const cone_descriptor_t*)desc);
        case PRIMITIVE_TYPE_BOX:      return virtual_node_replace_primitive(*node, *(const box_descriptor_t*)desc);
        case PRIMITIVE_TYPE_MESH:     return virtual_node_replace_primitive(*node, *(const mesh_descriptor_t*)desc);
        case PRIMITIVE_TYPE_EXTRUDE:  return virtual_node_replace_primitive(*node, *(const extrude_descriptor_t*)desc);
    }
}

EXTERN_C_END