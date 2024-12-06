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
        case PRIMITIVE_TYPE_EXTRUDE_POLYLINE: {
            extrude_polyline_descriptor_t* desc = static_cast<extrude_polyline_descriptor_t*>(prim.desc);
            for (int i = 0; i < desc->profile_number; i++) {
                free(desc->profiles[i].points);
                free(desc->profiles[i].bulges);
            }

            free(desc->axis.points);
            free(desc->axis.bulges);

            free(desc->profiles);

            break;
        }

        case PRIMITIVE_TYPE_EXTRUDE_ARCLINE: {
            extrude_arcline_descriptor_t* desc = static_cast<extrude_arcline_descriptor_t*>(prim.desc);
            for (int i = 0; i < desc->profile_number; i++) {
                free(desc->profiles[i].points);
                free(desc->profiles[i].bulges);
            }

            free(desc->profiles);

            break;
        }

        case PRIMITIVE_TYPE_EXTRUDE_HELIXLINE: {
            extrude_arcline_descriptor_t* desc = static_cast<extrude_arcline_descriptor_t*>(prim.desc);
            for (int i = 0; i < desc->profile_number; i++) {
                free(desc->profiles[i].points);
                free(desc->profiles[i].bulges);
            }

            free(desc->profiles);

            break;
        }
        default: break;
    }
    free(prim.desc);
}