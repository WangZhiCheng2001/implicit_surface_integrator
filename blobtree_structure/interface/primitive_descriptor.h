#pragma once
#include <macros.h>
#include <iostream>

typedef struct _raw_vector3d_t {
    double x, y, z;
} raw_vector3d_t;

typedef enum { constant, plane, sphere, cylinder, cone, box, mesh, extrude } descriptor;

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

typedef struct _cylinder_descriptor_t {
    raw_vector3d_t bottom_origion;
    double         radius;
    raw_vector3d_t offset;
} cylinder_descriptor_t;

typedef struct _cone_descriptor_t {
    raw_vector3d_t top_point;
    raw_vector3d_t bottom_point;
    double         radius1;
    double         radius2;
} cone_descriptor_t;

typedef struct _box_descriptor_t {
    raw_vector3d_t left_bottom_point;
    double         length;
    double         width;
    double         height;
} box_descriptor_t;

typedef struct _mesh_descriptor_t {
    int             face_number;
    raw_vector3d_t* points;
    int*            indexs;
    int**           faces;
} mesh_descriptor_t;

typedef struct _extrude_descriptor_t {
    int             edges_number;
    raw_vector3d_t  extusion;
    raw_vector3d_t* points;
    double*         bulges;
} extrude_descriptor_t;

EXTERN_C double evaluate_constant(constant_descriptor_t* desc, raw_vector3d_t point);
EXTERN_C double evaluate_plane(plane_descriptor_t* desc, raw_vector3d_t point);
EXTERN_C double evaluate_sphere(sphere_descriptor_t* desc, raw_vector3d_t point);
EXTERN_C double evaluate_cylinder(cylinder_descriptor_t* desc, raw_vector3d_t point);
EXTERN_C double evaluate_cone(cone_descriptor_t* desc, raw_vector3d_t point);
EXTERN_C double evaluate_box(box_descriptor_t* desc, raw_vector3d_t point);
EXTERN_C double evaluate_mesh(mesh_descriptor_t* desc, raw_vector3d_t point);
EXTERN_C double evaluate_extrude(extrude_descriptor_t* desc, raw_vector3d_t point);

#define evaluate(desc, point)                       \
    _Generic((desc),                                \
        constant_descriptor_t *: evaluate_constant, \
        plane_descriptor_t *: evaluate_plane,       \
        sphere_descriptor_t *: evaluate_sphere,     \
        cylinder_descriptor_t *: evaluate_cylinder, \
        cone_descriptor_t *: evaluate_cone,         \
        box_descriptor_t *: evaluate_box,           \
        mesh_descriptor_t *: evaluate_mesh,         \
        extrude_descriptor_t *: evaluate_extrude)(desc, point)
