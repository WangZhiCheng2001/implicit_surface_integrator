#pragma once

#include <stdint.h>

typedef struct {
    double x, y, z;
} raw_vector3d_t;

typedef enum {
    PRIMITIVE_TYPE_CONSTANT,
    PRIMITIVE_TYPE_PLANE,
    PRIMITIVE_TYPE_SPHERE,
    PRIMITIVE_TYPE_CYLINDER,
    PRIMITIVE_TYPE_CONE,
    PRIMITIVE_TYPE_BOX,
    PRIMITIVE_TYPE_MESH,
    PRIMITIVE_TYPE_EXTRUDE
} primitive_type;

typedef struct {
    double value;
} constant_descriptor_t;

typedef struct {
    raw_vector3d_t point;
    raw_vector3d_t normal;
} plane_descriptor_t;

typedef struct {
    raw_vector3d_t center;
    double         radius;
} sphere_descriptor_t;

typedef struct {
    raw_vector3d_t bottom_origion;
    double         radius;
    raw_vector3d_t offset;
} cylinder_descriptor_t;

typedef struct {
    raw_vector3d_t top_point;
    raw_vector3d_t bottom_point;
    double         radius1;
    double         radius2;
} cone_descriptor_t;

typedef struct {
    raw_vector3d_t center;
    raw_vector3d_t half_size;
} box_descriptor_t;

typedef struct {
    uint32_t        face_number;
    raw_vector3d_t* points;
    uint32_t*       indexs;
    uint32_t**      faces;
} mesh_descriptor_t;

typedef struct {
    uint32_t        edges_number;
    raw_vector3d_t  extusion;
    raw_vector3d_t* points;
    double*         bulges;
} extrude_descriptor_t;