#include "globals.hpp"
#include "internal_api.hpp"
#include "primitive_descriptor.h"
#include <iostream>

inline auto vec3d_conversion(const raw_vector3d_t& p) { return Eigen::Map<const Eigen::Vector3d>(&p.x); }

inline double sign(const double t) { return t >= 0.0 ? 1.0 : -1.0; }

inline double triangle_sdf(const Eigen::Ref<const Eigen::Vector3d>& p,
                           const Eigen::Ref<const Eigen::Vector3d>& a,
                           const Eigen::Ref<const Eigen::Vector3d>& b,
                           const Eigen::Ref<const Eigen::Vector3d>& c,
                           Eigen::Vector3d*                         closest_point = nullptr)
{
    auto ba  = b - a;
    auto pa  = p - a;
    auto cb  = c - b;
    auto pb  = p - b;
    auto ac  = a - c;
    auto pc  = p - c;
    auto nor = ba.cross(ac);

    auto test1     = pa.dot(ba.cross(nor));
    auto test2     = pb.dot(cb.cross(nor));
    auto test3     = pc.dot(ac.cross(nor));
    auto test_flag = sign(test1) + sign(test2) + sign(test3) < 2.0;
    if (test_flag) {
        Eigen::Vector3d c1 = a + ba * std::clamp(ba.dot(pa) / ba.squaredNorm(), 0.0, 1.0);
        Eigen::Vector3d c2 = b + cb * std::clamp(cb.dot(pb) / cb.squaredNorm(), 0.0, 1.0);
        Eigen::Vector3d c3 = c + ac * std::clamp(ac.dot(pc) / ac.squaredNorm(), 0.0, 1.0);

        double val1 = (c1 - p).norm();
        double val2 = (c2 - p).norm();
        double val3 = (c3 - p).norm();

        if (val1 <= val2 && val1 <= val3) {
            if (closest_point != nullptr) { *closest_point = c1; }
            return val1;
        } else if (val2 <= val3) {
            if (closest_point != nullptr) { *closest_point = c2; }
            return val2;
        } else {
            if (closest_point != nullptr) { *closest_point = c3; }
            return val3;
        }
    } else {
        double d = pa.dot(nor) / nor.norm();
        if (closest_point != nullptr) { *closest_point = p - nor.normalized() * d; }
        return std::abs(d);
    }
}

inline bool ray_intersects_triangle(const Eigen::Ref<const Eigen::Vector3d>& point,
                                    const Eigen::Ref<const Eigen::Vector3d>& dir,
                                    const Eigen::Ref<const Eigen::Vector3d>& v0,
                                    const Eigen::Ref<const Eigen::Vector3d>& v1,
                                    const Eigen::Ref<const Eigen::Vector3d>& v2)
{
    auto e1    = v1 - v0;
    auto e2    = v2 - v0;
    auto s     = point - v0;
    auto s1    = dir.cross(e2);
    auto s2    = s.cross(e1);
    auto coeff = 1.0 / s1.dot(e1);
    auto t     = coeff * s2.dot(e2);
    auto b1    = coeff * s1.dot(s);
    auto b2    = coeff * s2.dot(dir);
    return t >= 0 && b1 >= 0 && b2 >= 0 && (1 - b1 - b2) >= 0;
}

static const auto x_direction = Eigen::Vector3d{1.0, 0.0, 0.0};

// =========================================================================================================================

double evaluate(const constant_descriptor_t&             desc,
                const Eigen::Ref<const Eigen::Vector3d>& point,
                Eigen::Vector3d*                         closest_point = nullptr)
{
    if (closest_point != nullptr) { *closest_point = point; }
    return desc.value;
}

double evaluate(const plane_descriptor_t&                desc,
                const Eigen::Ref<const Eigen::Vector3d>& point,
                Eigen::Vector3d*                         closest_point = nullptr)
{
    auto normal   = vec3d_conversion(desc.normal);
    auto distance = normal.dot(point - vec3d_conversion(desc.point));

    if (closest_point != nullptr) { *closest_point = point - distance * normal; }
    return distance;
}

double evaluate(const sphere_descriptor_t&               desc,
                const Eigen::Ref<const Eigen::Vector3d>& point,
                Eigen::Vector3d*                         closest_point = nullptr)
{
    auto   center             = vec3d_conversion(desc.center);
    double distance_to_center = (point - center).norm();
    auto   distance_to_face   = distance_to_center - desc.radius;

    if (closest_point != nullptr) { *closest_point = center + (desc.radius / distance_to_center) * (point - center); }
    return distance_to_face;
}

double evaluate(const cylinder_descriptor_t&             desc,
                const Eigen::Ref<const Eigen::Vector3d>& point,
                Eigen::Vector3d*                         closest_point = nullptr)
{
    auto        bottom_center = vec3d_conversion(desc.bottom_origion);
    auto        offset        = vec3d_conversion(desc.offset);
    const auto& radius        = desc.radius;

    Eigen::Vector3d ba   = -offset;
    Eigen::Vector3d pa   = point - (bottom_center + offset);
    auto            baba = ba.squaredNorm();
    auto            paba = pa.dot(ba);
    auto            x    = (pa * baba - ba * paba).norm() - radius * baba;
    auto            y    = abs(paba - baba * 0.5) - baba * 0.5;
    auto            x2   = x * x;
    auto            y2   = y * y * baba;
    auto            d    = (std::max(x, y) < 0.0) ? -std::min(x2, y2) : (((x > 0.0) ? x2 : 0.0) + ((y > 0.0) ? y2 : 0.0));

    if (closest_point != nullptr) {
        Eigen::Vector3d result;
        auto            top_center = bottom_center + offset;
        auto            projection = bottom_center + offset + (paba / baba) * ba;
        auto            center     = bottom_center + 0.5 * offset;

        if (x < 0.0) {
            // Close to side
            if (y < 0.0 && x2 < y2) {
                result = projection + (point - projection).normalized() * radius;
            }
            // Close to top or bottom
            else {
                result = center + (projection - center).normalized() * offset.norm() * 0.5 + (point - projection);
            }
        } else // x >= 0.0
        {
            // Close to side
            if (y < 0.0) {
                result = projection + (point - projection).normalized() * radius;
            }
            // Close to (side and bottom) or (side and top)
            else // y >= 0.0
            {
                result = center + (projection - center).normalized() * offset.norm() * 0.5
                         + (point - projection).normalized() * radius;
            }
        }

#ifdef _DEBUG
        if (abs((point - result).norm() - (std::sqrt(abs(d)) / baba)) > 1e-8) {
            std::cerr << "Check fail of the closest point in cylinder" << std::endl;
            std::cerr << "Distace1: " << (point - result).norm() << std::endl;
            std::cerr << "Distace2: " << std::sqrt(abs(d)) / baba << std::endl;
            std::cerr << "Result Point: " << point.x() << "," << point.y() << "," << point.z() << std::endl;
            std::cerr << "Cylinder: " << std::endl;
            std::cerr << "Bottom: " << desc.bottom_origion.x << "," << desc.bottom_origion.y << "," << desc.bottom_origion.z
                      << std::endl;
            std::cerr << "Offset: " << desc.offset.x << "," << desc.offset.y << "," << desc.offset.z << std::endl;
            std::cerr << "Radius: " << desc.radius << std::endl << std::endl;
        }
#endif // _DEBUG
        *closest_point = result;
    }
    return sign(d) * std::sqrt(abs(d)) / baba;
}

double evaluate(const cone_descriptor_t&                 desc,
                const Eigen::Ref<const Eigen::Vector3d>& point,
                Eigen::Vector3d*                         closest_point = nullptr)
{
    Eigen::Vector3d ba = vec3d_conversion(desc.bottom_point) - vec3d_conversion(desc.top_point);
    Eigen::Vector3d pa = point - vec3d_conversion(desc.top_point);

    auto rba  = desc.radius2 - desc.radius1;
    auto baba = ba.squaredNorm();
    auto paba = pa.dot(ba) / baba;
    auto papa = pa.squaredNorm();
    auto x    = std::sqrt(papa - paba * paba * baba);
    auto cax  = std::max(0.0, x - ((paba < 0.5) ? desc.radius1 : desc.radius2));
    auto cay  = abs(paba - 0.5) - 0.5;
    auto k    = rba * rba + baba;
    auto f    = std::clamp((rba * (x - desc.radius1) + paba * baba) / k, 0.0, 1.0);
    auto cbx  = x - desc.radius1 - f * rba;
    auto cby  = paba - f;
    auto s    = (cbx < 0.0 && cay < 0.0) ? -1.0 : 1.0;

    auto distance1 = cax * cax + cay * cay * baba;
    auto distance2 = cbx * cbx + cby * cby * baba;

    if (closest_point != nullptr) {
        Eigen::Vector3d result;

        auto top    = vec3d_conversion(desc.top_point);
        auto bottom = vec3d_conversion(desc.bottom_point);

        auto projection = vec3d_conversion(desc.top_point) + paba * ba;
        auto center     = (vec3d_conversion(desc.bottom_point) + vec3d_conversion(desc.top_point)) * 0.5;

        // bottom or top
        if (distance1 < distance2) {
            // top
            if ((point - top).squaredNorm() < (point - bottom).squaredNorm()) {
                // inner
                if (cax == 0.0) {
                    result = top + (point - projection);
                }
                // outer
                else {
                    result = top + (point - projection).normalized() * desc.radius1;
                }
            }
            // bottom
            else {
                // inner
                if (cax == 0.0) {
                    result = bottom + (point - projection);
                }
                // outer
                else {
                    result = bottom + (point - projection).normalized() * desc.radius2;
                }
            }
        }
        // side
        else {
            // inner
            if (cbx < 0.0) {
                result = point + (point - projection).normalized() * abs(cbx) - ba * abs(cby);
            }
            // outer
            else {
                result = point - (point - projection).normalized() * abs(cbx) + ba * abs(cby);
            }
        }

#ifdef _DEBUG
        if (abs((point - result).norm() - (std::sqrt(std::min(distance1, distance2)))) > 1e-8) {
            std::cerr << "Check fail of the closest point in cone" << std::endl;
            std::cerr << "Distace1: " << (point - result).norm() << std::endl;
            std::cerr << "Distace2: " << std::sqrt(std::min(distance1, distance2)) << std::endl;
            std::cerr << "Result Point: " << point.x() << "," << point.y() << "," << point.z() << std::endl;
            std::cerr << "Cone: " << std::endl;
            std::cerr << "Bottom: " << desc.bottom_point.x << "," << desc.bottom_point.y << "," << desc.bottom_point.z
                      << std::endl;
            std::cerr << "Top: " << desc.top_point.x << "," << desc.top_point.y << "," << desc.top_point.z << std::endl;
            std::cerr << "Radius1: " << desc.radius1 << std::endl << std::endl;
            std::cerr << "Radius2: " << desc.radius2 << std::endl << std::endl;
        }
#endif // _DEBUG
        *closest_point = result;
    }

    return s * std::sqrt(std::min(distance1, distance2));
}

double evaluate(const box_descriptor_t&                  desc,
                const Eigen::Ref<const Eigen::Vector3d>& point,
                Eigen::Vector3d*                         closest_point = nullptr)
{
    // HINT: this method is not ACCURATE for OUTSIDE OF THE BOX, but it saves time
    auto            center    = vec3d_conversion(desc.center);
    auto            half_size = vec3d_conversion(desc.half_size);
    Eigen::Vector3d d         = (point - center).cwiseAbs() - half_size;

#ifdef _DEBUG
    auto distance = d.cwiseMax(0.0).norm() + std::min(d.maxCoeff(), 0.0);
#else
    auto distance = d.maxCoeff();
#endif // _DEBUG

    if (closest_point != nullptr) {
        Eigen::Vector3d result;
        auto            distance_to_center      = (point - center).cwiseAbs();
        auto            distance_to_center_sign = (point - center).cwiseSign();

        auto distance_to_face      = (distance_to_center.cwiseAbs() - half_size).cwiseAbs();
        auto distance_to_face_sign = (distance_to_center.cwiseAbs() - half_size).cwiseSign();

        auto flag = distance_to_face_sign.sum();

        if (flag < -1.9) // flag = -3.0, -2.0
        {
            int min_distance_index;
            distance_to_face.minCoeff(&min_distance_index);

            auto temp = distance_to_center_sign.cwiseProduct(distance_to_face);

            result                      = point;
            result[min_distance_index] += temp[min_distance_index];
        } else if (flag < 0.1) // flag == -1.0, 0.0
        {
            int max_flag_index;
            distance_to_face_sign.maxCoeff(&max_flag_index);

            auto temp = -distance_to_center_sign.cwiseProduct(distance_to_face);

            result                  = point;
            result[max_flag_index] += temp[max_flag_index];
        } else if (flag < 1.1) // flag == 1.0
        {
            int min_flag_index;
            distance_to_face_sign.minCoeff(&min_flag_index);

            result                 = center + distance_to_center_sign.cwiseProduct(half_size);
            result[min_flag_index] = point[min_flag_index];
        } else // flag == 2.0, 3.0
        {
            result = center + distance_to_center_sign.cwiseProduct(half_size);
        }

#ifdef _DEBUG
        if (abs((point - result).norm() - distance) > 1e-8) {
            std::cerr << "Check fail of the closest point in box" << std::endl;
            std::cerr << "Distace1: " << (point - result).norm() << std::endl;
            std::cerr << "Distace2: " << distance << std::endl;
            std::cerr << "Result Point: " << point.x() << "," << point.y() << "," << point.z() << std::endl;
            std::cerr << "Box: " << std::endl;
            std::cerr << "Center: " << desc.center.x << "," << desc.center.y << "," << desc.center.z << std::endl;
            std::cerr << "Half Size: " << desc.half_size.x << "," << desc.half_size.y << "," << desc.half_size.z << std::endl;
        }
#endif // _DEBUG
        *closest_point = result;
    }

    return distance;
}

double evaluate(const mesh_descriptor_t&                 desc,
                const Eigen::Ref<const Eigen::Vector3d>& point,
                Eigen::Vector3d*                         closest_point = nullptr)
{
    // Note: There is no check for out-of-bounds access to points, indexes and faces
    auto points  = desc.points;
    auto indices = desc.indices;
    auto face    = desc.faces;

    double   min_distance{std::numeric_limits<double>::infinity()};
    uint32_t count{};

    int min_index1 = 0;
    int min_index2 = 0;

    for (auto i = 0; i < desc.face_number; i++) {
        const auto& begin_index = face[i].begin_index;
        const auto& length      = face[i].vertex_count;

        auto point0 = vec3d_conversion(points[indices[begin_index]]);
        bool flag{};
        for (auto j = 1; j < length - 1; j++) {
            auto point1 = vec3d_conversion(points[indices[begin_index + j]]);
            auto point2 = vec3d_conversion(points[indices[begin_index + j + 1]]);
            auto temp   = triangle_sdf(point, point0, point1, point2);

            if (temp < min_distance) {
                min_distance;
                min_index1 = i;
                min_index2 = j;
            }

            if (!flag && ray_intersects_triangle(point, x_direction, point0, point1, point2)) { flag = true; }
        }
        if (flag) { count++; }
    }

    if (closest_point != nullptr) {
        const auto& begin_index = face[min_index1].begin_index;
        const auto& length      = face[min_index1].vertex_count;

        auto point0 = vec3d_conversion(points[indices[begin_index]]);
        auto point1 = vec3d_conversion(points[indices[begin_index + min_index2]]);
        auto point2 = vec3d_conversion(points[indices[begin_index + min_index2 + 1]]);
        triangle_sdf(point, point0, point1, point2, closest_point);
    }

    if (min_distance < 1e-8) { return 0; }
    if (count % 2 == 1) {
        return -min_distance;
    } else {
        return min_distance;
    }
}

// =========================================================================================================================
#include "extrude/utils.hpp"

double evaluate(const extrude_polyline_descriptor_t&     desc,
                const Eigen::Ref<const Eigen::Vector3d>& point,
                Eigen::Vector3d*                         closest_point = nullptr)
{
    PolyLine              axis = to_polyline(desc.axis);
    std::vector<PolyLine> profiles;
    for (int i = 0; i < desc.profile_number; i++) { profiles.push_back(to_polyline(desc.profiles[i])); }

    ExtrudedSolidPolyline body{profiles, axis};
    const Vec3            query_point = Vec3(point.x(), point.y(), point.z());

    Vec3 close;
    auto result = body.sdf(query_point, close);

    if (closest_point != nullptr) { *closest_point = vec3d_conversion(vec3_to_raw(close)); }

    return result;
}

double evaluate(const extrude_arcline_descriptor_t&      desc,
                const Eigen::Ref<const Eigen::Vector3d>& point,
                Eigen::Vector3d*                         closest_point = nullptr)
{
    ArcLine               axis = to_arcline(desc.axis);
    std::vector<PolyLine> profiles;
    for (int i = 0; i < desc.profile_number; i++) { profiles.push_back(to_polyline(desc.profiles[i])); }

    ExtrudedSolidArcLine body{profiles, axis};
    const Vec3           query_point = Vec3(point.x(), point.y(), point.z());

    Vec3 close;
    auto result = body.sdf(query_point, close);

    if (closest_point != nullptr) { *closest_point = vec3d_conversion(vec3_to_raw(close)); }

    return result;
}

double evaluate(const extrude_helixline_descriptor_t&    desc,
                const Eigen::Ref<const Eigen::Vector3d>& point,
                Eigen::Vector3d*                         closest_point = nullptr)
{
    HelixLine             axis = to_helixline(desc.axis);
    std::vector<PolyLine> profiles;
    for (int i = 0; i < desc.profile_number; i++) { profiles.push_back(to_polyline(desc.profiles[i])); }

    ExtrudedSolidHelixLine body{profiles, axis};

    Vec3       close;
    const Vec3 query_point = Vec3(point.x(), point.y(), point.z());
    auto       result      = body.sdf(query_point, close);

    if (closest_point != nullptr) { *closest_point = vec3d_conversion(vec3_to_raw(close)); }

    return result;
}

// =========================================================================================================================

BPE_API double evaluate(uint32_t index, const Eigen::Ref<const Eigen::Vector3d>& point)
{
    const auto& primitive = primitives[index];
    switch (primitive.type) {
        case PRIMITIVE_TYPE_CONSTANT:          return evaluate(*(const constant_descriptor_t*)primitive.desc, point);
        case PRIMITIVE_TYPE_PLANE:             return evaluate(*(const plane_descriptor_t*)primitive.desc, point);
        case PRIMITIVE_TYPE_SPHERE:            return evaluate(*(const sphere_descriptor_t*)primitive.desc, point);
        case PRIMITIVE_TYPE_CYLINDER:          return evaluate(*(const cylinder_descriptor_t*)primitive.desc, point);
        case PRIMITIVE_TYPE_CONE:              return evaluate(*(const cone_descriptor_t*)primitive.desc, point);
        case PRIMITIVE_TYPE_BOX:               return evaluate(*(const box_descriptor_t*)primitive.desc, point);
        case PRIMITIVE_TYPE_MESH:              return evaluate(*(const mesh_descriptor_t*)primitive.desc, point);
        case PRIMITIVE_TYPE_EXTRUDE_POLYLINE:  return evaluate(*(const extrude_polyline_descriptor_t*)primitive.desc, point);
        case PRIMITIVE_TYPE_EXTRUDE_ARCLINE:   return evaluate(*(const extrude_arcline_descriptor_t*)primitive.desc, point);
        case PRIMITIVE_TYPE_EXTRUDE_HELIXLINE: return evaluate(*(const extrude_helixline_descriptor_t*)primitive.desc, point);
    }
}