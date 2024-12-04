#pragma once

#include "globals.hpp"

static inline void destroy_primitive_node(primitive_node_t& prim)
{
    switch (prim.type) {
        case PRIMITIVE_TYPE_MESH: {
            mesh_descriptor_t* desc = static_cast<mesh_descriptor_t*>(prim.desc);
            free(desc->points);
            free(desc->indices);
            free(desc->faces);
            break;
        }
        case PRIMITIVE_TYPE_EXTRUDE: {
            extrude_descriptor_t* desc = static_cast<extrude_descriptor_t*>(prim.desc);
            free(desc->points);
            free(desc->bulges);
            break;
        }
        default: break;
    }
    free(prim.desc);
}