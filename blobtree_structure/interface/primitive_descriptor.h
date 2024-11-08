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

// Placeholder, currently used to represent empty body
typedef struct {
    double value; // Use 0 to represent an empty body
} constant_descriptor_t;

// Plane descriptor
typedef struct {
    raw_vector3d_t point;  // The base point of the plane
    raw_vector3d_t normal; // The normal of the plane
} plane_descriptor_t;

// Sphere descriptor
typedef struct {
    raw_vector3d_t center; // The ceter of the sphere
    double         radius; // The radius of the sphere
} sphere_descriptor_t;

// Cylinder descriptor
typedef struct {
    raw_vector3d_t bottom_origion; // The origion of bottom face of the cylinder
    double         radius;         // The radius of the cylinder
    raw_vector3d_t offset;         // The vector from the origion of bottom face to the origion of the top face
} cylinder_descriptor_t;

typedef struct {
    raw_vector3d_t top_point;    // The origion of top face of the cone
    raw_vector3d_t bottom_point; // The origion of bottom face of the cone
    double         radius1;      // The radius of the top face
    double         radius2;      // The radius of the bottom face
} cone_descriptor_t;

typedef struct {
    raw_vector3d_t center;    // The center of the box
    raw_vector3d_t half_size; // The { half_length, half_width, half_height} of the box
} box_descriptor_t;

// Mesh descriptor
typedef struct {
    uint32_t        point_number; // The point number of the mesh
    uint32_t        face_number;  // The face number of the mesh
    raw_vector3d_t* points;       // The points of the mesh
    uint32_t*       indexs;       // The indexs from face to point
    uint32_t**      faces;        // Two-dimensional array, Use [begin index, length] to represent a face
} mesh_descriptor_t;

// Extrude descriptor
typedef struct {
    uint32_t        edges_number; // The edge number in the bottom face
    raw_vector3d_t  extusion;     // The offset vector
    raw_vector3d_t* points;       // The points of the bottom face
    double*         bulges;       // The bulge of each edge
} extrude_descriptor_t;