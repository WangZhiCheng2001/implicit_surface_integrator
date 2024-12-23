#pragma once

#include <limits>
#define _USE_MATH_DEFINES
#include <cmath>

#include <internal_primitive_desc.hpp>
#include <algorithm/glue_algorithm.hpp>

namespace internal
{
// HINT: all normal points inside the boundary
// TODO: add detect when evaluating, and directly return iff given point is on the line

// =============================================================
// structures
// =============================================================

struct line_closest_param_t {
    Eigen::Vector3d point{};
    double          t{-1.};
    double          distance{std::numeric_limits<double>::max()};
    bool            is_peak_value{true};

    friend inline bool operator<(const line_closest_param_t& lhs, const line_closest_param_t& rhs);
};

struct line_simple_distance_param_t {
    double t{};
    bool   need_refine{false};
    bool   is_peak_value{true};
};

inline bool operator<(const line_closest_param_t& lhs, const line_closest_param_t& rhs) { return lhs.distance < rhs.distance; }

// =============================================================
// interpolate methods
// =============================================================

[[nodiscard]] static inline Eigen::Vector2d line_interpolate(const Eigen::Vector2d* p_start, const double t)
{
    return (1 - t) * *p_start + t * (*(p_start + 1));
}

[[nodiscard]] static inline Eigen::Vector2d line_interpolate_derivative(const Eigen::Vector2d* p_start)
{
    return (*(p_start + 1) - *p_start).normalized();
}

// HINT: assume the plane normal (ref_normal) to be {0, 0, 1}, then the normal should be this
[[nodiscard]] static inline Eigen::Vector2d line_interpolate_normal(const Eigen::Vector2d* p_start)
{
    const auto line_vec = line_interpolate_derivative(p_start);
    return Eigen::Vector2d(line_vec[1], -line_vec[0]);
}

// HINT: here p must be transformed into the local plane
[[nodiscard]] static inline line_closest_param_t line_closest_param(const Eigen::Vector2d* p_start, const Eigen::Vector3d& p)
{
    const auto line_vec = *(p_start + 1) - *p_start;
    const auto p_dir    = p.topRows<2>() - *p_start;

    // illegal solve, return null
    const auto line_vec_length_2 = line_vec.squaredNorm();
    const auto raw_t             = line_vec.dot(p_dir);
    if (raw_t < 0 || raw_t > line_vec_length_2) return {};

    const auto p_dir_length_2 = p_dir.squaredNorm();
    const auto t              = raw_t / line_vec_length_2;
    return {
        {(1 - t) * p_start->x() + t * (p_start + 1)->x(), (1 - t) * p_start->y() + t * (p_start + 1)->y(), 0},
        t,
        std::sqrt(p_dir_length_2 - raw_t * raw_t / line_vec_length_2 + p[2] * p[2])
    };
}

[[nodiscard]] static inline Eigen::Vector2d circle_interpolate(const Eigen::Vector2d* p_start,
                                                               const double           theta,
                                                               const double           t)
{
    // const auto sin_theta = std::sin(theta);
    // const auto alpha     = std::sin(t * theta) / sin_theta;
    // const auto beta      = std::sin((1 - t) * theta) / sin_theta;
    // return *p_start * beta + *(p_start + 2) * alpha + *(p_start + 1) * (1 - alpha - beta);

    // i.e. new method uses polar coordinates
    const auto alpha = std::cos(t * theta);
    const auto beta  = std::sin(t * theta);
    return alpha * *p_start + beta * *(p_start + 2) + (1 - alpha - beta) * *(p_start + 1);
}

[[nodiscard]] static inline Eigen::Vector2d circle_interpolate_derivative(const Eigen::Vector2d* p_start,
                                                                          const double           theta,
                                                                          const double           t)
{
    // const auto sin_theta   = std::sin(theta);
    // const auto alpha_deriv = std::cos(t * theta) / sin_theta;
    // const auto beta_deriv  = std::cos((1 - t) * theta) / sin_theta;
    // return (*p_start * -beta_deriv + *(p_start + 2) * alpha_deriv + *(p_start + 1) * (beta_deriv -
    // alpha_deriv)).normalized();

    const auto alpha_deriv = -std::sin(t * theta);
    const auto beta_deriv  = std::cos(t * theta);
    return (alpha_deriv * *p_start + beta_deriv * *(p_start + 2) - (alpha_deriv + beta_deriv) * *(p_start + 1)).normalized();
}

[[nodiscard]] static inline Eigen::Vector2d circle_interpolate_normal(const Eigen::Vector2d* p_start,
                                                                      const double           theta,
                                                                      const double           t)
{
    // const auto sin_theta = std::sin(theta);
    // const auto alpha     = std::sin(t * theta) / sin_theta;
    // const auto beta      = std::sin((1 - t) * theta) / sin_theta;
    // return (*(p_start + 1) * (alpha + beta) - *p_start * beta - *(p_start + 2) * alpha).normalized();

    const auto alpha = std::cos(t * theta);
    const auto beta  = std::sin(t * theta);
    return (alpha + beta) * *(p_start + 1) - alpha * *p_start - beta * *(p_start + 2);
}

[[nodiscard]] static inline line_closest_param_t circle_closest_param(const Eigen::Vector2d* p_start,
                                                                      const double           theta,
                                                                      const Eigen::Vector3d& p)
{
    const auto base_vec1 = *p_start - *(p_start + 1);
    const auto base_vec2 = *(p_start + 2) - *(p_start + 1);
    const auto p_vec     = p.topRows<2>() - *(p_start + 1);

    const auto local_x = base_vec1.dot(p_vec);
    const auto local_y = base_vec2.dot(p_vec);
    const auto phi     = std::atan2(local_y, local_x);
    // illegal solve, return null
    if (phi < 0 || phi > theta) return {};

    const auto p_vec_norm   = p_vec.norm();
    const auto r            = base_vec1.norm();
    const auto dis_on_plane = p_vec_norm - r;
    return {
        {p_vec / p_vec_norm * r + *(p_start + 1), 0},
        phi / theta,
        std::sqrt(p[2] * p[2] + dis_on_plane * dis_on_plane)
    };
}

// =============================================================
// derived functions
// =============================================================

static constexpr auto EPSILON    = std::numeric_limits<double>::epsilon() * 1e6;
static constexpr auto TWO_PI     = M_PI * 2;
static constexpr auto PI2        = M_PI / 2;
static constexpr auto INV_PI     = 1. / M_PI;
static constexpr auto INV_TWO_PI = 1. / TWO_PI;

[[nodiscard]] static inline Eigen::Vector2d evaluate(const polyline& line, double t)
{
    assert(t >= 0 && t <= line.start_indices.size());

    double     frac;
    const auto n = static_cast<uint32_t>(std::modf(t, &frac));
    if (line.thetas[n] <= EPSILON)
        return line_interpolate(&line.vertices[line.start_indices[n]], frac);
    else
        return circle_interpolate(&line.vertices[line.start_indices[n]], line.thetas[n], frac);
}

[[nodiscard]] static inline Eigen::Vector2d calculate_normal(const polyline& line, double t)
{
    assert(t >= 0 && t <= line.start_indices.size());

    double     frac;
    const auto n = static_cast<uint32_t>(std::modf(t, &frac));
    if (line.thetas[n] <= EPSILON)
        return line_interpolate_normal(&line.vertices[line.start_indices[n]]);
    else
        return circle_interpolate_normal(&line.vertices[line.start_indices[n]], line.thetas[n], frac);
}

[[nodiscard]] static inline Eigen::Vector2d calculate_tangent(const polyline& line, double t)
{
    assert(t >= 0 && t <= line.start_indices.size());

    double     frac;
    const auto n = static_cast<uint32_t>(std::modf(t, &frac));
    if (line.thetas[n] <= EPSILON)
        return line_interpolate_derivative(&line.vertices[line.start_indices[n]]);
    else
        return circle_interpolate_derivative(&line.vertices[line.start_indices[n]], line.thetas[n], frac);
}

[[nodiscard]] static inline line_closest_param_t calculate_closest_param(const polyline& line, const Eigen::Vector3d& p)
{
    std::vector<line_closest_param_t> closest_params(line.thetas.size());
    algorithm::transform(counting_iterator<size_t>(),
                         counting_iterator<size_t>(line.thetas.size()),
                         closest_params.begin(),
                         [&](size_t index) {
                             const auto& theta = line.thetas[index];
                             if (theta <= EPSILON)
                                 return line_closest_param(&line.vertices[line.start_indices[index]], p);
                             else
                                 return circle_closest_param(&line.vertices[line.start_indices[index]], theta, p);
                         });

    const auto iter = algorithm::min_element(closest_params.begin(), closest_params.end());
    if (iter->t >= 0) { // i.e. has closest point prependicular to the line, which is the best solution
        iter->t += std::distance(closest_params.begin(), iter);
        return *iter;
    } else {            // in this case, the closest point can only be the vertices of the line
        algorithm::transform(counting_iterator<size_t>(),
                             counting_iterator<size_t>(line.thetas.size()),
                             closest_params.begin(),
                             [&](size_t index) {
                                 const auto q    = evaluate(line, static_cast<double>(index));
                                 const auto dist = std::sqrt((p.topRows<2>() - q).squaredNorm() + p[2] * p[2]);
                                 return line_closest_param_t{
                                     {q.x(), q.y(), 0},
                                     static_cast<double>(index),
                                     dist,
                                     false
                                 };
                             });
        return *algorithm::min_element(closest_params.begin(), closest_params.end());
    }
}

[[nodiscard]] static inline Eigen::Vector3d evaluate(const helixline& line, double t)
{
    assert(t >= 0 && t <= 1);

    // return line.start_point + t * line.height * line.axis_direction
    //        + line.radius * std::cos(t * line.total_theta) * line.base_direction_u
    //        + line.radius * std::sin(t * line.total_theta) * line.base_direction_v;
    return {line.radius * std::cos(t * line.total_theta), line.radius * std::sin(t * line.total_theta), t * line.height};
}

[[nodiscard]] static inline Eigen::Vector3d calculate_normal(const helixline& line, double t)
{
    assert(t >= 0 && t <= 1);

    return {-std::cos(t * line.total_theta), -std::sin(t * line.total_theta), 0.};
}

[[nodiscard]] static inline Eigen::Vector3d calculate_tangent(const helixline& line, double t)
{
    assert(t >= 0 && t <= 1);

    // auto circle_part = line.base_direction_v * std::cos(t * line.total_theta) //
    //                    - line.base_direction_u * std::sin(t * line.total_theta);
    // return (line.height * line.axis_direction + line.total_theta * line.radius * circle_part).normalized();
    return Eigen::Vector3d{-line.total_theta * line.radius * std::sin(t * line.total_theta),
                           line.total_theta * line.radius * std::cos(t * line.total_theta),
                           line.height}
        .normalized();
}

[[nodiscard]] static inline line_closest_param_t calculate_closest_param(const helixline& line, const Eigen::Vector3d& p)
{
    const auto h_p             = p.z();
    const auto theta_p         = std::atan2(p.y(), p.x());
    const auto r_p             = p.topRows<2>().norm();
    const auto theta_intersect = line.total_theta * h_p / line.height;
    const auto k               = (theta_intersect - theta_p) * INV_TWO_PI;
    const auto rounded_k       = std::round(k);

    // early exit case: point p is on the helix
    if (std::abs(k - rounded_k) <= EPSILON) {
        const auto theta = theta_p + rounded_k * TWO_PI;
        if (theta >= 0 && theta <= line.total_theta)
            return {p, theta / line.total_theta, 0., true};
        else {
            // for points two sides away from the helix, this may cause slight numerical error
            // but it should be fine since this kind of points are rare
            const auto clipped_theta = std::clamp(theta, 0., line.total_theta);
            const auto t             = clipped_theta / line.total_theta;
            const auto closest_point = evaluate(line, t);
            const auto dist          = (closest_point - p).norm();
            return {std::move(closest_point), std::move(t), std::move(dist), false};
        }
    }

    const auto times_p             = -theta_p * (1 + line.total_theta) * INV_PI;
    const auto times_line          = line.total_theta * line.total_theta * INV_PI;
    const auto rounded_times_start = std::ceil(times_p - 0.5);
    const auto rounded_times_end   = std::floor(times_line + times_p - 0.5);

    const auto line_theta_r   = line.total_theta * line.radius;
    const auto inv_line_theta = 1. / line.total_theta;
    const auto t_intersect    = std::sqrt((theta_intersect * theta_intersect * line.radius * line.radius + h_p * h_p)
                                       / (line_theta_r * line_theta_r + line.height * line.height));

    // collect all peak values of f(t) in domain [0, 1], and then find the interval which:
    // 1. has one root as extreme minimum point
    // 2. the root has smallest distance to t_intersect
    line_simple_distance_param_t peak_value_param{};
    {
        const auto                             alpha = line.total_theta * line.radius * r_p;
        std::vector<std::pair<double, double>> peak_values(rounded_times_end - rounded_times_start + 1 + 2);
        peak_values.front() = {.0, -line_theta_r * r_p * std::sin(theta_p) - h_p};
        peak_values.back()  = {1., line_theta_r * r_p * std::sin(line.total_theta - theta_p) + line.height - h_p};
        algorithm::transform(counting_iterator<size_t>(rounded_times_start),
                             counting_iterator<size_t>(rounded_times_end + 1),
                             peak_values.begin() + 1,
                             [&](size_t k_) -> std::pair<double, double> {
                                 const auto t = (theta_p + PI2 + k_ * M_PI) * inv_line_theta;
                                 return {std::move(t),
                                         alpha * std::sin(t * line.total_theta - theta_p) + t * line.height - h_p};
                             });
        peak_value_param = algorithm::transform_reduce(
            counting_iterator<size_t>{},
            counting_iterator<size_t>{peak_values.size() - 1},
            line_simple_distance_param_t{},
            [&](const line_simple_distance_param_t& lhs, const line_simple_distance_param_t& rhs) {
                return std::abs(lhs.t - t_intersect) < std::abs(rhs.t - t_intersect);
            },
            [&](size_t index) -> line_simple_distance_param_t {
                const auto& [t0, f0] = peak_values[index];
                const auto& [t1, f1] = peak_values[index + 1];
                bool increasing_flag = (f0 > 0 && f1 > 0);
                bool decreasing_flag = (f0 < 0 && f1 < 0);
                // i.e. has a root t=t0, or distance is always increasing as t increases
                if ((f0 >= -EPSILON && f0 <= EPSILON) || increasing_flag) return {t0, false, !increasing_flag};
                // i.e. has a root t=t1, or distance is always decreasing as t increases
                if ((f1 >= -EPSILON && f1 <= EPSILON) || decreasing_flag) return {t1, false, !decreasing_flag};
                // i.e. has a root t in (0, 1)
                // in this case, we use linear interpolation as a guess
                return {f0 / (f0 - f1), true, true};
            });
    }

    // if we need to refine the guess, just do it!!!
    if (peak_value_param.need_refine) {
        auto&      t = peak_value_param.t;
        double     delta_t{std::numeric_limits<double>::max()};
        const auto alpha       = line.total_theta * line.radius * r_p;
        const auto alpha_deriv = alpha * line.total_theta;
        while (std::abs(delta_t) >= EPSILON) {
            // origin equation: f(t)=theta_all*r_all*r_p*sin(t*theta_all-theta_p)+t*height_all-h_p=0
            // derivative: f'(t)=theta_all^2*r_all*r_p*cos(t*theta_all-theta_p)+height_all=0
            const auto f        = alpha * std::sin(t * line.total_theta - theta_p) + t * line.height - h_p;
            const auto f_deriv  = alpha_deriv * std::cos(t * line.total_theta - theta_p) + line.height;
            delta_t             = f / f_deriv;
            t                  -= delta_t;
        }
    }

    // get the final result
    const auto closest_point = evaluate(line, peak_value_param.t);
    const auto dist          = (closest_point - p).norm();
    return {std::move(closest_point), peak_value_param.t, dist, peak_value_param.is_peak_value};


    // if (true) // complex case: we have at least one (peak) minimum point in domain [0, 1]
    // {
    //     // see if we can get an initial guess in domain t \in [0, 1]
    //     const auto domain_start_theta    = -h_p * line.height / line_theta_r_r;
    //     const auto domain_theta_interval = line_total_length_square / line.height;
    //     const auto domain_k_start        = std::round((domain_start_theta - theta_p) * INV_TWO_PI);
    //     const auto domain_k_end          = std::round((domain_start_theta + domain_theta_interval - theta_p) * INV_TWO_PI);

    //     double t_init;
    //     if (domain_k_start <= domain_k_end) {
    //         // we can get at least one initial guess
    //         const auto selected_k  = std::clamp(rounded_k, domain_k_start, domain_k_end);
    //         const auto theta_guess = theta_p + selected_k * TWO_PI;
    //         t_init                 = (theta_guess * line.radius * line_theta_r + h_p * line.height)
    //                  / (line_theta_r * line_theta_r + line.height * line.height);
    //     } else {
    //         // we got global minimum in domain [0, 1], but has no initial guess in domain [0, 1]
    //         // that means we only have one minimum point, use linear interpolation as initial guess
    //         const auto f0 = -line_theta_r * r_p * std::sin(theta_p) - h_p;
    //         const auto f1 = line_theta_r * r_p * std::sin(line.total_theta - theta_p) + line.height - h_p;
    //         t_init        = f0 / (f0 - f1);
    //     }

    //     // use Newton method to refine the guess
    //     double     t{t_init}, delta_t{std::numeric_limits<double>::max()};
    //     const auto alpha       = line.total_theta * line.radius * r_p;
    //     const auto alpha_deriv = alpha * line.total_theta;
    //     while (std::abs(delta_t) >= EPSILON) {
    //         // origin equation: f(t)=theta_all*r_all*r_p*sin(t*theta_all-theta_p)+t*height_all-h_p=0
    //         // derivative: f'(t)=theta_all^2*r_all*r_p*cos(t*theta_all-theta_p)+height_all=0
    //         const auto f        = alpha * std::sin(t * line.total_theta - theta_p) + t * line.height - h_p;
    //         const auto f_deriv  = alpha_deriv * std::cos(t * line.total_theta - theta_p) + line.height;
    //         delta_t             = f / f_deriv;
    //         t                  -= delta_t;
    //     }

    //     const auto closest_point = t * line.height * line.axis_direction                                  //
    //                                + line.radius * std::cos(t * line.total_theta) * line.base_direction_u //
    //                                + line.radius * std::sin(t * line.total_theta) * line.base_direction_v;
    //     return {closest_point, t, (p_vec - closest_point).norm(), true};
    // } else {
    //     // f(t) is always increasing or decreasing in domain [0, 1]
    //     // which means that closest point is either the start or end point of the helix
    // }
}
} // namespace internal