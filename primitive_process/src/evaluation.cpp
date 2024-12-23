#include <internal_api.hpp>

#include "primitive_process.hpp"

// =========================================================================================================================

struct evaluation_routine_tag;
struct closest_point_routine_tag;

template <typename T>
static constexpr bool is_process_routine_tag_v = false;

template <>
static constexpr bool is_process_routine_tag_v<evaluation_routine_tag> = true;

template <>
static constexpr bool is_process_routine_tag_v<closest_point_routine_tag> = true;

template <typename T>
static constexpr bool is_evaluation_routine_v = std::is_same_v<T, evaluation_routine_tag>;

template <typename T>
static constexpr bool is_closest_point_routine_v = std::is_same_v<T, closest_point_routine_tag>;

// =========================================================================================================================

inline auto vec3d_conversion(const raw_vector3d_t& p) { return Eigen::Map<const Eigen::Vector3d>(&p.x); }

inline double sign(const double t) { return t >= 0.0 ? 1.0 : -1.0; }

template <typename Routine, typename = std::enable_if_t<is_process_routine_tag_v<Routine>>>
inline auto triangle_sdf(Routine&&                                tag,
                         const Eigen::Ref<const Eigen::Vector3d>& p,
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

    Eigen::Vector3d test_vals = {sign(pa.dot(ba.cross(nor))), //
                                 sign(pb.dot(cb.cross(nor))), //
                                 sign(pc.dot(ac.cross(nor)))};
    if (test_vals.sum() < 2.0) {
        std::array closest_points = {a + ba * std::clamp(ba.dot(pa) / ba.squaredNorm(), 0.0, 1.0),
                                     b + cb * std::clamp(cb.dot(pb) / cb.squaredNorm(), 0.0, 1.0),
                                     c + ac * std::clamp(ac.dot(pc) / ac.squaredNorm(), 0.0, 1.0)};
        std::array distance = {(closest_points[0] - p).norm(), (closest_points[1] - p).norm(), (closest_points[2] - p).norm()};
        auto       min_iter = std::min_element(distance.begin(), distance.end());
        if constexpr (is_evaluation_routine_v<Routine>)
            return *min_iter;
        else
            return closest_points[std::distance(distance.begin(), min_iter)];
    } else {
        auto distance = pa.dot(nor) / nor.norm();
        if constexpr (is_evaluation_routine_v<Routine>)
            return std::abs(distance);
        else
            return p - distance * nor.normalized();
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

PE_API double evaluate(const constant_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point) { return desc.value; }

PE_API double evaluate(const plane_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point)
{
    return vec3d_conversion(desc.normal).dot(point - vec3d_conversion(desc.point));
}

PE_API double evaluate(const sphere_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point)
{
    return (point - vec3d_conversion(desc.center)).norm() - desc.radius;
}

PE_API double evaluate(const cylinder_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point)
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

PE_API double evaluate(const cone_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point)
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

PE_API double evaluate(const box_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point)
{
    // HINT: this method is not ACCURATE for OUTSIDE OF THE BOX, but it saves time
    auto            center    = vec3d_conversion(desc.center);
    auto            half_size = vec3d_conversion(desc.half_size);
    Eigen::Vector3d d         = (point - center).cwiseAbs() - half_size;
    return d.maxCoeff();
}

PE_API double evaluate(const mesh_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point)
{
    // Note: There is no check for out-of-bounds access to points, indexes and faces
    auto points  = desc.points;
    auto indices = desc.indices;
    auto face    = desc.faces;

    double   min_distance{std::numeric_limits<double>::infinity()};
    uint32_t count{};

    for (auto i = 0; i < desc.face_number; i++) {
        const auto& begin_index = face[i].begin_index;
        const auto& length      = face[i].vertex_count;

        auto point0 = vec3d_conversion(points[indices[begin_index]]);
        bool flag{};
        for (auto j = 1; j < length - 1; j++) {
            auto point1  = vec3d_conversion(points[indices[begin_index + j]]);
            auto point2  = vec3d_conversion(points[indices[begin_index + j + 1]]);
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

PE_API double evaluate(uint32_t index, const Eigen::Ref<const Eigen::Vector3d>& point)
{
    const auto& primitive = get_primitive_node(index);
    switch (primitive.type) {
        case PRIMITIVE_TYPE_CONSTANT: return evaluate(*(const constant_descriptor_t*)primitive.desc, point);
        case PRIMITIVE_TYPE_PLANE:    return evaluate(*(const plane_descriptor_t*)primitive.desc, point);
        case PRIMITIVE_TYPE_SPHERE:   return evaluate(*(const sphere_descriptor_t*)primitive.desc, point);
        case PRIMITIVE_TYPE_CYLINDER: return evaluate(*(const cylinder_descriptor_t*)primitive.desc, point);
        case PRIMITIVE_TYPE_CONE:     return evaluate(*(const cone_descriptor_t*)primitive.desc, point);
        case PRIMITIVE_TYPE_BOX:      return evaluate(*(const box_descriptor_t*)primitive.desc, point);
        case PRIMITIVE_TYPE_MESH:     return evaluate(*(const mesh_descriptor_t*)primitive.desc, point);
        case PRIMITIVE_TYPE_EXTRUDE:  return evaluate(*(const extrude_descriptor_t*)primitive.desc, point);
    }
}