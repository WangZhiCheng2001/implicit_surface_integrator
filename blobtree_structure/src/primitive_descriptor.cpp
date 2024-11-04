#include "primitive_descriptor.h"

typedef raw_vector3d_t vec3;

vec3 add(const vec3& point1, const vec3& point2) { return vec3{point1.x + point2.x, point1.y + point2.y, point1.z + point2.z}; }

vec3 operator+(const vec3& point1, const vec3& point2) { return add(point1, point2); }

vec3 sub(const vec3& point1, const vec3& point2) { return vec3{point1.x - point2.x, point1.y - point2.y, point1.z - point2.z}; }

vec3 operator-(const vec3& point1, const vec3& point2) { return sub(point1, point2); }

vec3 mul(const vec3& vector, const double scalar) { return vec3{vector.x * scalar, vector.y * scalar, vector.z * scalar}; }

vec3 operator*(const vec3& point1, const double scalar) { return mul(point1, scalar); }

vec3 div(const vec3& vector, const double scalar)
{
    if (scalar == 0) { throw std::runtime_error("Division by zero error."); }
    return vec3{vector.x / scalar, vector.y / scalar, vector.z / scalar};
}

vec3 operator/(const vec3& point1, const double scalar) { return div(point1, scalar); }

double dot(const vec3& vector1, const vec3& vector2)
{
    return vector1.x * vector2.x + vector1.y * vector2.y + vector1.z * vector2.z;
}

double dot2(const vec3& vector) { return dot(vector, vector); }

vec3 cross(const vec3& vector1, const vec3& vector2)
{
    return vec3{vector1.y * vector2.z - vector1.z * vector2.y,
                vector1.z * vector2.x - vector1.x * vector2.z,
                vector1.x * vector2.y - vector1.y * vector2.x};
}

double len(const vec3& vector) { return sqrt(dot(vector, vector)); }

double dis(const vec3& point1, const vec3& point2) { return len(sub(point1, point2)); }

double clamp(const double t, const double min, const double max)
{
    if (t <= min) { return min; }
    if (t >= max) { return max; }
    return t;
}

double sign(const double t) { return t >= 0.0 ? 1.0 : -1.0; }

vec3 normalize(const vec3& vector)
{
    double temp = len(vector);
    if (abs(temp) < 1e-8) { throw std::runtime_error("Cannot normalize a zero-length vector."); }
    temp = 1.0 / temp;
    return vec3{vector.x * temp, vector.y * temp, vector.z * temp};
}

double evaluate_constant(constant_descriptor_t* desc, raw_vector3d_t point) { return desc->value; }

double evaluate_plane(plane_descriptor_t* desc, raw_vector3d_t point) { return dot(point - desc->point, desc->normal); }

double evaluate_sphere(sphere_descriptor_t* desc, raw_vector3d_t point) { return dis(point, desc->center) - desc->radius; }

double evaluate_cylinder(cylinder_descriptor_t* desc, raw_vector3d_t point)
{
    vec3&  b = desc->bottom_origion;
    vec3   a = b + desc->offset;
    vec3&  p = point;
    double r = desc->radius;

    vec3   ba   = b - a;
    vec3   pa   = p - a;
    double baba = dot(ba, ba);
    double paba = dot(pa, ba);
    double x    = len(pa * baba - ba * paba) - r * baba;
    double y    = abs(paba - baba * 0.5) - baba * 0.5;
    double x2   = x * x;
    double y2   = y * y * baba;
    double d    = (fmax(x, y) < 0.0) ? -fmin(x2, y2) : (((x > 0.0) ? x2 : 0.0) + ((y > 0.0) ? y2 : 0.0));
    return sign(d) * sqrt(abs(d)) / baba;
}

double evaluate_cone(cone_descriptor_t* desc, raw_vector3d_t point)
{
    vec3&  a  = desc->top_point;
    vec3&  b  = desc->bottom_point;
    vec3&  p  = point;
    double ra = desc->radius1;
    double rb = desc->radius2;

    double rba  = rb - ra;
    double baba = dot(b - a, b - a);
    double papa = dot(p - a, p - a);
    double paba = dot(p - a, b - a) / baba;
    double x    = sqrt(papa - paba * paba * baba);
    double cax  = fmax(0.0, x - ((paba < 0.5) ? ra : rb));
    double cay  = abs(paba - 0.5) - 0.5;
    double k    = rba * rba + baba;
    double f    = clamp((rba * (x - ra) + paba * baba) / k, 0.0, 1.0);
    double cbx  = x - ra - f * rba;
    double cby  = paba - f;
    double s    = (cbx < 0.0 && cay < 0.0) ? -1.0 : 1.0;
    return s * sqrt(fmin(cax * cax + cay * cay * baba, cbx * cbx + cby * cby * baba));
}

double evaluate_box(box_descriptor_t* desc, raw_vector3d_t point)
{
    // Get the minimum and maximum bounding coordinates of the box
    auto min_point = desc->left_bottom_point;
    auto max_point = min_point + vec3{desc->length, desc->width, desc->height};

    // Point in the box
    if (point.x >= min_point.x && point.x <= max_point.x && point.y >= min_point.y && point.y <= max_point.y
        && point.z >= min_point.z && point.z <= max_point.z) {
        double min = fmin(point.x - min_point.x, max_point.x - point.x);
        min        = fmin(min, fmin(point.y - min_point.y, max_point.y - point.y));
        min        = fmin(min, fmin(point.z - min_point.y, max_point.z - point.z));
        return -min;
    } else {
        // Calculate the closest distance from the point to the border of each dimension of the box
        double dx = fmax(fmax(min_point.x - point.x, point.x - max_point.x), 0.0);
        double dy = fmax(fmax(min_point.y - point.y, point.y - max_point.y), 0.0);
        double dz = fmax(fmax(min_point.z - point.z, point.z - max_point.z), 0.0);
        return sqrt(dx * dx + dy * dy + dz * dz);
    }
}

double triangle_sdf(const vec3& p, const vec3& a, const vec3& b, const vec3& c)
{
    vec3 ba  = b - a;
    vec3 pa  = p - a;
    vec3 cb  = c - b;
    vec3 pb  = p - b;
    vec3 ac  = a - c;
    vec3 pc  = p - c;
    vec3 nor = cross(ba, ac);

    return sqrt((sign(dot(cross(ba, nor), pa)) + sign(dot(cross(cb, nor), pb)) + sign(dot(cross(ac, nor), pc)) < 2.0)
                    ? fmin(fmin(dot2(ba * clamp(dot(ba, pa) / dot2(ba), 0.0, 1.0) - pa),
                                dot2(cb * clamp(dot(cb, pb) / dot2(cb), 0.0, 1.0) - pb)),
                           dot2(ac * clamp(dot(ac, pc) / dot2(ac), 0.0, 1.0) - pc))
                    : dot(nor, pa) * dot(nor, pa) / dot2(nor));
}

bool ray_intersects_triangle(const vec3& point, const vec3& dir, const vec3& v0, const vec3& v1, const vec3& v2)
{
    vec3   e1    = v1 - v0;
    vec3   e2    = v2 - v0;
    vec3   s     = point - v0;
    vec3   s1    = cross(dir, e2);
    vec3   s2    = cross(s, e1);
    double coeff = 1.0 / dot(s1, e1);
    double t     = coeff * dot(s2, e2);
    double b1    = coeff * dot(s1, s);
    double b2    = coeff * dot(s2, dir);
    return t >= 0 && b1 >= 0 && b2 >= 0 && (1 - b1 - b2) >= 0;
}

double evaluate_mesh(mesh_descriptor_t* desc, raw_vector3d_t point)
{
    // Note: There is no check for out-of-bounds access to points, indexes and faces
    auto points = desc->points;
    auto indexs = desc->indexs;
    auto face   = desc->faces;

    double min_distance = std::numeric_limits<double>::infinity();
    int    count        = 0;
    for (int i = 0; i < desc->face_number; i++) {
        int begin_index = face[i][0];
        int length      = face[i][1];

        auto& point0 = points[indexs[begin_index]];
        bool  flag   = false;
        for (int j = 1; j < length - 1; j++) {
            double temp  = triangle_sdf(point, point0, points[indexs[j]], points[indexs[j + 1]]);
            min_distance = fmin(min_distance, temp);
            if (!flag
                && ray_intersects_triangle(point, vec3{1.0, 0.0, 0.0}, point0, points[indexs[j]], points[indexs[j + 1]])) {
                flag = true;
            }
        }
        if (flag) { count++; }
    }

    if (min_distance < 1e-8) { return 0; }
    if (count % 2 == 1) {
        return -min_distance;
    } else {
        return min_distance;
    }
}

double evaluate_extrude(extrude_descriptor_t* desc, raw_vector3d_t point)
{
    // Note: There is no check for out-of-bounds access to points and bulges
    auto points   = desc->points;
    auto bulges   = desc->bulges;
    auto extusion = desc->extusion;

    double min_distance = std::numeric_limits<double>::infinity();
    int    count        = 0;

    // Note: Currently only straight edges are considered, the bottom and top surfaces are polygons
    auto& point0 = points[0];
    bool  flag1  = false;
    bool  flag2  = false;
    for (int i = 1; i < desc->edges_number - 1; i++) {
        // Bottom
        double temp  = triangle_sdf(point, point0, points[i], points[i + 1]);
        min_distance = fmin(min_distance, temp);
        if (!flag1 && ray_intersects_triangle(point, vec3{1.0, 0.0, 0.0}, point0, points[i], points[i + 1])) { flag1 = true; }

        // Top
        temp         = triangle_sdf(point, point0 + extusion, points[i] + extusion, points[i + 1] + extusion);
        min_distance = fmin(min_distance, temp);
        if (!flag2
            && ray_intersects_triangle(point,
                                       vec3{1.0, 0.0, 0.0},
                                       point0 + extusion,
                                       points[i] + extusion,
                                       points[i + 1] + extusion)) {
            flag2 = true;
        }
    }
    if (flag1) { count++; }
    if (flag2) { count++; }

    // Side
    for (int i = 0; i < desc->edges_number; i++) {
        auto& point1 = points[i];
        vec3  point2;
        if (i + 1 == desc->edges_number) {
            point2 = points[0];
        } else {
            point2 = points[i + 1];
        }
        auto point3 = point2 + extusion;
        auto point4 = point1 + extusion;

        auto bulge = bulges[i];
        if (abs(bulge) < 1e-8) {
            // Straight Edge
            bool flag = false;

            double temp  = triangle_sdf(point, point1, point2, point3);
            min_distance = fmin(min_distance, temp);
            if (!flag && ray_intersects_triangle(point, vec3{1.0, 0.0, 0.0}, point1, point2, point3)) { flag = true; }

            temp         = triangle_sdf(point, point1, point3, point4);
            min_distance = fmin(min_distance, temp);
            if (!flag && ray_intersects_triangle(point, vec3{1.0, 0.0, 0.0}, point1, point3, point4)) { flag = true; }

            if (flag) { count++; }
        } else {
            // Curved Edge
            // TODO
        }
    }
    if (count % 2 == 1) {
        return -min_distance;
    } else {
        return min_distance;
    }
}
