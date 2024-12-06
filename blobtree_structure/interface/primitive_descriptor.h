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
    PRIMITIVE_TYPE_EXTRUDE_POLYLINE,
    PRIMITIVE_TYPE_EXTRUDE_ARCLINE,
    PRIMITIVE_TYPE_EXTRUDE_HELIXLINE
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
    uint32_t begin_index;
    uint32_t vertex_count;
} polygon_face_descriptor_t;

typedef struct {
    uint32_t                   point_number; // The point number of the mesh
    uint32_t                   face_number;  // The face number of the mesh
    raw_vector3d_t*            points;       // The points of the mesh
    uint32_t*                  indices;      // The indices from face to point
    polygon_face_descriptor_t* faces;        // Two-dimensional array, Use [begin index, length] to represent a face
} mesh_descriptor_t;

// Extrude descriptor
typedef struct {
    uint32_t        point_number;     // The point number of the polyline
    raw_vector3d_t* points;           // The point of the polyline
    uint32_t        bulge_number;     // The bulge number of the polyline
    double*         bulges;           // The bulge of each edge
    raw_vector3d_t  reference_normal; // The reference normal of the polyline
    bool            is_close;         // Whether the polyline is close
} polyline_descriptor_t;

typedef struct {
    raw_vector3d_t start;            // The start point of the arc line
    raw_vector3d_t end;              // The end point of the arc line
    double         bulge;            // The bugle of the arc line
    raw_vector3d_t reference_normal; // The reference normal of the arcline
} arcline_descriptor_t;

typedef struct {
    raw_vector3d_t axis_start;        // The start point of the helix line
    raw_vector3d_t axis_end;          // The end point of the helix line
    double         radius;            // The radius of the helix line
    double         advance_per_round; // he advance per round of the helix line
    raw_vector3d_t start_direction;   // The direction from axisStart to start of the helix line
} helixline_descriptor_t;

// Note : In profiles, The first polyline is outer boundary, and the ohters is internal holes

typedef struct {
    int                    profile_number; // The profiles number of the extruded solid
    polyline_descriptor_t* profiles;       // The profiles of the extruded solid
    polyline_descriptor_t  axis;           // The axis of the extruded solid
} extrude_polyline_descriptor_t;

typedef struct {
    int                    profile_number; // The profiles number of the extruded solid
    polyline_descriptor_t* profiles;       // The profiles of the extruded solid
    arcline_descriptor_t   axis;           // The axis of the extruded solid
} extrude_arcline_descriptor_t;

typedef struct {
    int                    profile_number; // The profiles number of the extruded solid
    polyline_descriptor_t* profiles;       // The profiles of the extruded solid
    helixline_descriptor_t axis;           // The axis of the extruded solid
} extrude_helixline_descriptor_t;
