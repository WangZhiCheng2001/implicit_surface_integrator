#pragma once

#include <macros.h>

typedef struct _raw_vector3d_t {
    double x, y, z;
} raw_vector3d_t;

typedef struct _constant_descriptor_t {
    double value;
} constant_descriptor_t;

typedef struct _plane_descriptor_t {
    raw_vector3d_t point;
    raw_vector3d_t normal;
} plane_descriptor_t;

typedef struct _sphere_descriptor_t {
    raw_vector3d_t center;
    double         radius;
} sphere_descriptor_t;

EXTERN_C API double evaluate_constant(constant_descriptor_t* desc, raw_vector3d_t point);
EXTERN_C API double evaluate_plane(plane_descriptor_t* desc, raw_vector3d_t point);
EXTERN_C API double evaluate_sphere(sphere_descriptor_t* desc, raw_vector3d_t point);

#define evaluate(desc, point)                       \
    _Generic((desc),                                \
        constant_descriptor_t *: evaluate_constant, \
        plane_descriptor_t *: evaluate_plane,       \
        sphere_descriptor_t *: evaluate_sphere)(desc, point)