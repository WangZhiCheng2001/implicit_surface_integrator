#include <internal_api.hpp>

inline auto vec3d_conversion(const raw_vector3d_t& p) { return Eigen::Map<const Eigen::Vector3d>(&p.x); }

inline double sign(const double t) { return t >= 0.0 ? 1.0 : -1.0; }

inline double triangle_sdf(const Eigen::Ref<const Eigen::Vector3d>& p,
                           const Eigen::Ref<const Eigen::Vector3d>& a,
                           const Eigen::Ref<const Eigen::Vector3d>& b,
                           const Eigen::Ref<const Eigen::Vector3d>& c)
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
        auto val1 = (ba * std::clamp(ba.dot(pa) / ba.squaredNorm(), 0.0, 1.0) - pa).norm();
        auto val2 = (cb * std::clamp(cb.dot(pb) / cb.squaredNorm(), 0.0, 1.0) - pb).norm();
        auto val3 = (ac * std::clamp(ac.dot(pc) / ac.squaredNorm(), 0.0, 1.0) - pc).norm();
        return std::min({val1, val2, val3});
    } else {
        return pa.dot(nor) / nor.norm();
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

BPE_API double evaluate(const constant_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point)
{
    return desc.value;
}

BPE_API double evaluate(const plane_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point)
{
    return vec3d_conversion(desc.normal).dot(point - vec3d_conversion(desc.point));
}

BPE_API double evaluate(const sphere_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point)
{
    return (point - vec3d_conversion(desc.center)).norm() - desc.radius;
}

BPE_API double evaluate(const cylinder_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point)
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
    return sign(d) * std::sqrt(abs(d)) / baba;
}

BPE_API double evaluate(const cone_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point)
{
    Eigen::Vector3d ba = vec3d_conversion(desc.bottom_point) - vec3d_conversion(desc.top_point);
    Eigen::Vector3d pa = point - vec3d_conversion(desc.bottom_point);

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
    return s * std::sqrt(std::min(cax * cax + cay * cay * baba, cbx * cbx + cby * cby * baba));
}

BPE_API double evaluate(const box_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point)
{
    // HINT: this method is not ACCURATE for OUTSIDE OF THE BOX, but it saves time
    auto            center    = vec3d_conversion(desc.center);
    auto            half_size = vec3d_conversion(desc.half_size);
    Eigen::Vector3d d         = (point - center).cwiseAbs() - half_size;
    return d.maxCoeff();
}

BPE_API double evaluate(const mesh_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point)
{
    // Note: There is no check for out-of-bounds access to points, indexes and faces
    auto points = desc.points;
    auto indexs = desc.indexs;
    auto face   = desc.faces;

    double   min_distance{std::numeric_limits<double>::infinity()};
    uint32_t count{};

    for (auto i = 0; i < desc.face_number; i++) {
        int begin_index = face[i][0];
        int length      = face[i][1];

        auto point0 = vec3d_conversion(points[indexs[begin_index]]);
        bool flag{};
        for (auto j = 1; j < length - 1; j++) {
            auto point1  = vec3d_conversion(points[indexs[begin_index + j]]);
            auto point2  = vec3d_conversion(points[indexs[begin_index + j + 1]]);
            auto temp    = triangle_sdf(point, point0, point1, point2);
            min_distance = std::min(min_distance, temp);
            if (!flag && ray_intersects_triangle(point, x_direction, point0, point1, point2)) { flag = true; }
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

BPE_API double evaluate(const extrude_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point)
{
    // Note: There is no check for out-of-bounds access to points and bulges
    auto points   = desc.points;
    auto bulges   = desc.bulges;
    auto extusion = vec3d_conversion(desc.extusion);

    double   min_distance{std::numeric_limits<double>::infinity()};
    uint32_t count{};

    // Note: Currently only straight edges are considered, the bottom and top surfaces are polygons
    auto point0 = vec3d_conversion(points[0]);
    bool flag1{}, flag2{};
    for (auto i = 1; i < desc.edges_number - 1; i++) {
        auto   point1 = vec3d_conversion(points[i]);
        auto   point2 = vec3d_conversion(points[i + 1]);
        // Bottom
        double temp   = triangle_sdf(point, point0, point1, point2);
        min_distance  = std::min(min_distance, temp);
        if (!flag1 && ray_intersects_triangle(point, x_direction, point0, point1, point2)) { flag1 = true; }

        // Top
        temp         = triangle_sdf(point, point0 + extusion, point1 + extusion, point2 + extusion);
        min_distance = std::min(min_distance, temp);
        if (!flag2 && ray_intersects_triangle(point, x_direction, point0 + extusion, point1 + extusion, point2 + extusion)) {
            flag2 = true;
        }
    }
    if (flag1) { count++; }
    if (flag2) { count++; }

    // Side
    for (auto i = 0; i < desc.edges_number; i++) {
        auto point1 = vec3d_conversion(points[i]);
        auto point2 = (i + 1 == desc.edges_number) ? point0 : vec3d_conversion(points[i + 1]);
        auto point3 = point2 + extusion;
        auto point4 = point1 + extusion;

        auto bulge = bulges[i];
        if (abs(bulge) < 1e-8) {
            // Straight Edge
            bool flag = false;

            double temp  = triangle_sdf(point, point1, point2, point3);
            min_distance = fmin(min_distance, temp);
            if (!flag && ray_intersects_triangle(point, x_direction, point1, point2, point3)) { flag = true; }

            temp         = triangle_sdf(point, point1, point3, point4);
            min_distance = fmin(min_distance, temp);
            if (!flag && ray_intersects_triangle(point, x_direction, point1, point3, point4)) { flag = true; }

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