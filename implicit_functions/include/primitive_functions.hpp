#pragma once

#include "implicit_function.hpp"

static constexpr double kEpsilon = 1e-6;

template <typename Scalar, size_t Dim>
class ConstantFunction : public ImplicitFunction<Scalar, Dim>
{
public:
    explicit ConstantFunction(Scalar value) : value_(value) {}

    Scalar evaluate_scalar(const Eigen::Ref<const Eigen::Vector<Scalar, Dim>> &pos) const { return value_; }

    Eigen::Vector<Scalar, Dim> evaluate_gradient(const Eigen::Ref<const Eigen::Vector<Scalar, Dim>> &pos) const
    {
        return Eigen::Vector<Scalar, Dim>::Zero();
    }

    Eigen::Vector<Scalar, Dim + 1> evaluate_scalar_gradient(const Eigen::Ref<const Eigen::Vector<Scalar, Dim>> &pos) const
    {
        auto res = Eigen::Vector<Scalar, Dim + 1>::Zero();
        res[0]   = value_;
        return res;
    }

private:
    Scalar value_{};
};

template <typename Scalar, size_t Dim>
class PlaneDistanceFunction : public ImplicitFunction<Scalar, Dim>
{
public:
    PlaneDistanceFunction(const Eigen::Ref<const Eigen::Vector<Scalar, Dim>> &point,
                          const Eigen::Ref<const Eigen::Vector<Scalar, Dim>> &normal)
        : point_(point), normal_(normal)
    {
        normal_.normalize();
    }

    Scalar evaluate_scalar(const Eigen::Ref<const Eigen::Vector<Scalar, Dim>> &pos) const { return normal_.dot(pos - point_); }

    Eigen::Vector<Scalar, Dim> evaluate_gradient(const Eigen::Ref<const Eigen::Vector<Scalar, Dim>> &pos) const
    {
        return normal_;
    }

    Eigen::Vector<Scalar, Dim + 1> evaluate_scalar_gradient(const Eigen::Ref<const Eigen::Vector<Scalar, Dim>> &pos) const
    {
        Eigen::Vector<Scalar, Dim + 1> res{};
        res.template head<Dim>().array() = normal_.array();
        res[Dim]                         = evaluate_scalar(pos);
        return res;
    }

private:
    Eigen::Vector<Scalar, Dim> point_{};
    Eigen::Vector<Scalar, Dim> normal_{};
};

template <typename Scalar, size_t Dim>
class SphereDistanceFunction : public ImplicitFunction<Scalar, Dim>
{
public:
    SphereDistanceFunction(const Eigen::Ref<const Eigen::Vector<Scalar, Dim>> &center, Scalar radius)
        : center_(center), radius_(radius)
    {
    }

    Scalar evaluate_scalar(const Eigen::Ref<const Eigen::Vector<Scalar, Dim>> &pos) const
    {
        return -((pos - center_).norm() - radius_);
    }

    Eigen::Vector<Scalar, Dim> evaluate_gradient(const Eigen::Ref<const Eigen::Vector<Scalar, Dim>> &pos) const
    {
        auto   vec  = pos - center_;
        Scalar norm = vec.norm();
        if (norm <= kEpsilon) return Eigen::Vector<Scalar, Dim>::Zero();
        return vec / norm;
    }

    Eigen::Vector<Scalar, Dim + 1> evaluate_scalar_gradient(const Eigen::Ref<const Eigen::Vector<Scalar, Dim>> &pos) const
    {
        Eigen::Vector<Scalar, Dim + 1> res{};

        auto   vec  = pos - center_;
        Scalar norm = vec.norm();
        if (norm <= kEpsilon) {
            res[Dim] = -radius_;
            return res;
        }

        res.template head<Dim>().array() = vec.array() / norm;
        res[Dim]                         = norm - radius_;
        return res;
    }

private:
    Eigen::Vector<Scalar, Dim> center_{};
    Scalar                     radius_{};
};

template <typename Scalar, size_t Dim>
class CylinderDistanceFunction;

template <typename Scalar>
class CylinderDistanceFunction<Scalar, 3> : public ImplicitFunction<Scalar, 3>
{
public:
    CylinderDistanceFunction(const Eigen::Ref<const Eigen::Vector<Scalar, 3>> &axis_point,
                             const Eigen::Ref<const Eigen::Vector<Scalar, 3>> &axis_dir,
                             Scalar                                            radius)
        : axis_point_(axis_point), axis_dir_(axis_dir), radius_(radius)
    {
        axis_dir_.normalize();
    }

    Scalar evaluate_scalar(const Eigen::Ref<const Eigen::Vector<Scalar, 3>> &pos) const
    {
        auto vec  = pos - axis_point_;
        auto dist = vec.dot(axis_dir_);
        return (vec - dist * axis_dir_).norm() - radius_;
    }

    Eigen::Vector<Scalar, 3> evaluate_gradient(const Eigen::Ref<const Eigen::Vector<Scalar, 3>> &pos) const
    {
        auto                     vec      = pos - axis_point_;
        auto                     dist     = vec.dot(axis_dir_);
        auto                     vec_para = dist * axis_dir_;
        Eigen::Vector<Scalar, 3> vec_perp = vec - vec_para;
        Scalar                   norm     = vec_perp.norm();
        if (norm <= kEpsilon) return Eigen::Vector<Scalar, 3>::Zero();
        return vec_perp / norm;
    }

    Eigen::Vector<Scalar, 4> evaluate_scalar_gradient(const Eigen::Ref<const Eigen::Vector<Scalar, 3>> &pos) const
    {
        Eigen::Vector<Scalar, 4> res{};

        auto                     vec      = pos - axis_point_;
        auto                     dist     = vec.dot(axis_dir_);
        auto                     vec_para = dist * axis_dir_;
        Eigen::Vector<Scalar, 3> vec_perp = vec - vec_para;
        Scalar                   norm     = vec_perp.norm();
        if (norm <= kEpsilon) {
            res[3] = -radius_;
            return res;
        }

        res.template head<3>().array() = vec_perp.array() / norm;
        res[3]                         = norm - radius_;
        return res;
    }

private:
    Eigen::Vector<Scalar, 3> axis_point_{};
    Eigen::Vector<Scalar, 3> axis_dir_{};
    Scalar                   radius_{};
};

template <typename Scalar, size_t Dim>
class ConeDistanceFunction;

template <typename Scalar>
class ConeDistanceFunction<Scalar, 3> : public ImplicitFunction<Scalar, 3>
{
public:
    ConeDistanceFunction(const Eigen::Ref<const Eigen::Vector<Scalar, 3>> &apex_point,
                         const Eigen::Ref<const Eigen::Vector<Scalar, 3>> &axis_dir,
                         Scalar                                            apex_angle)
        : apex_point_(apex_point), axis_dir_(axis_dir), apex_angle_cosine_(std::cos(apex_angle))
    {
        axis_dir_.normalize();
    }

    Scalar evaluate_scalar(const Eigen::Ref<const Eigen::Vector<Scalar, 3>> &pos) const
    {
        return apex_angle_cosine_ * (pos - apex_point_).norm() - (pos - apex_point_).dot(axis_dir_);
    }

    Eigen::Vector<Scalar, 3> evaluate_gradient(const Eigen::Ref<const Eigen::Vector<Scalar, 3>> &pos) const
    {
        auto vec_apex = pos - apex_point_;
        auto vec_norm = vec_apex.norm();
        if (vec_norm <= kEpsilon) return Eigen::Vector<Scalar, 3>::Zero();
        return vec_apex * apex_angle_cosine_ / vec_norm - axis_dir_;
    }

    Eigen::Vector<Scalar, 4> evaluate_scalar_gradient(const Eigen::Ref<const Eigen::Vector<Scalar, 3>> &pos) const
    {
        auto vec_apex = pos - apex_point_;
        auto vec_norm = vec_apex.norm();
        if (vec_norm <= kEpsilon) return Eigen::Vector<Scalar, 4>::Zero();

        Eigen::Vector<Scalar, 4> res{};
        res.template head<3>().array() = vec_apex.array() * apex_angle_cosine_ / vec_norm - axis_dir_.array();
        res[3]                         = apex_angle_cosine_ * vec_norm - vec_apex.dot(axis_dir_);
        return res;
    }

private:
    Eigen::Vector<Scalar, 3> apex_point_{};
    Eigen::Vector<Scalar, 3> axis_dir_{};
    Scalar                   apex_angle_cosine_{};
};

template <typename Scalar, size_t Dim>
class TorusDistanceFunction;