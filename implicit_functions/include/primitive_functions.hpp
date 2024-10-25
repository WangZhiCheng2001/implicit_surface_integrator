#pragma once

#include "implicit_function.hpp"

static constexpr double kEpsilon = 1e-6;

class ConstantFunction : public ImplicitFunction
{
public:
    explicit ConstantFunction(double value) : value_(value) {}

    template <size_t Dim>
    double evaluate_scalar(const Eigen::Ref<const Eigen::Vector<double, Dim>> &pos) const
    {
        return value_;
    }

    template <size_t Dim>
    Eigen::Vector<double, Dim> evaluate_gradient(const Eigen::Ref<const Eigen::Vector<double, Dim>> &pos) const
    {
        return Eigen::Vector<double, Dim>::Zero();
    }

    template <size_t Dim>
    Eigen::Vector<double, Dim + 1> evaluate_scalar_gradient(const Eigen::Ref<const Eigen::Vector<double, Dim>> &pos) const
    {
        auto res = Eigen::Vector<double, Dim + 1>::Zero();
        res[0]   = value_;
        return res;
    }

private:
    double value_{};
};

template <size_t Dim>
class PlaneDistanceFunction : public ImplicitFunction
{
public:
    PlaneDistanceFunction(const Eigen::Ref<const Eigen::Vector<double, Dim>> &point,
                          const Eigen::Ref<const Eigen::Vector<double, Dim>> &normal)
        : point_(point), normal_(normal)
    {
        normal_.normalize();
    }

    double evaluate_scalar(const Eigen::Ref<const Eigen::Vector<double, Dim>> &pos) const { return normal_.dot(pos - point_); }

    Eigen::Vector<double, Dim> evaluate_gradient(const Eigen::Ref<const Eigen::Vector<double, Dim>> &pos) const
    {
        return normal_;
    }

    Eigen::Vector<double, Dim + 1> evaluate_scalar_gradient(const Eigen::Ref<const Eigen::Vector<double, Dim>> &pos) const
    {
        Eigen::Vector<double, Dim + 1> res{};
        res.template head<Dim>().array() = normal_.array();
        res[Dim]                         = evaluate_scalar(pos);
        return res;
    }

private:
    Eigen::Vector<double, Dim> point_{};
    Eigen::Vector<double, Dim> normal_{};
};

template <size_t Dim>
class SphereDistanceFunction : public ImplicitFunction
{
public:
    SphereDistanceFunction(const Eigen::Ref<const Eigen::Vector<double, Dim>> &center, double radius)
        : center_(center), radius_(radius)
    {
    }

    double evaluate_scalar(const Eigen::Ref<const Eigen::Vector<double, Dim>> &pos) const
    {
        return (pos - center_).norm() - radius_;
    }

    Eigen::Vector<double, Dim> evaluate_gradient(const Eigen::Ref<const Eigen::Vector<double, Dim>> &pos) const
    {
        auto   vec  = pos - center_;
        double norm = vec.norm();
        if (norm <= kEpsilon) return Eigen::Vector<double, Dim>::Zero();
        return vec / norm;
    }

    Eigen::Vector<double, Dim + 1> evaluate_scalar_gradient(const Eigen::Ref<const Eigen::Vector<double, Dim>> &pos) const
    {
        Eigen::Vector<double, Dim + 1> res{};

        auto   vec  = pos - center_;
        double norm = vec.norm();
        if (norm <= kEpsilon) {
            res[Dim] = -radius_;
            return res;
        }

        res.template head<Dim>().array() = vec.array() / norm;
        res[Dim]                         = norm - radius_;
        return res;
    }

private:
    Eigen::Vector<double, Dim> center_{};
    double                     radius_{};
};

template <size_t Dim>
class CylinderDistanceFunction;

template <>
class CylinderDistanceFunction<3> : public ImplicitFunction
{
public:
    CylinderDistanceFunction(const Eigen::Ref<const Eigen::Vector3d> &axis_point,
                             const Eigen::Ref<const Eigen::Vector3d> &axis_dir,
                             double                                   radius)
        : axis_point_(axis_point), axis_dir_(axis_dir), radius_(radius)
    {
        axis_dir_.normalize();
    }

    double evaluate_scalar(const Eigen::Ref<const Eigen::Vector3d> &pos) const
    {
        auto vec  = pos - axis_point_;
        auto dist = vec.dot(axis_dir_);
        return (vec - dist * axis_dir_).norm() - radius_;
    }

    Eigen::Vector3d evaluate_gradient(const Eigen::Ref<const Eigen::Vector3d> &pos) const
    {
        auto            vec      = pos - axis_point_;
        auto            dist     = vec.dot(axis_dir_);
        auto            vec_para = dist * axis_dir_;
        Eigen::Vector3d vec_perp = vec - vec_para;
        double          norm     = vec_perp.norm();
        if (norm <= kEpsilon) return Eigen::Vector3d::Zero();
        return vec_perp / norm;
    }

    Eigen::Vector4d evaluate_scalar_gradient(const Eigen::Ref<const Eigen::Vector3d> &pos) const
    {
        Eigen::Vector4d res{};

        auto            vec      = pos - axis_point_;
        auto            dist     = vec.dot(axis_dir_);
        auto            vec_para = dist * axis_dir_;
        Eigen::Vector3d vec_perp = vec - vec_para;
        double          norm     = vec_perp.norm();
        if (norm <= kEpsilon) {
            res[3] = -radius_;
            return res;
        }

        res.template head<3>().array() = vec_perp.array() / norm;
        res[3]                         = norm - radius_;
        return res;
    }

private:
    Eigen::Vector3d axis_point_{};
    Eigen::Vector3d axis_dir_{};
    double          radius_{};
};

template <size_t Dim>
class ConeDistanceFunction;

template <>
class ConeDistanceFunction<3> : public ImplicitFunction
{
public:
    ConeDistanceFunction(const Eigen::Ref<const Eigen::Vector3d> &apex_point,
                         const Eigen::Ref<const Eigen::Vector3d> &axis_dir,
                         double                                   apex_angle)
        : apex_point_(apex_point), axis_dir_(axis_dir), apex_angle_cosine_(std::cos(apex_angle))
    {
        axis_dir_.normalize();
    }

    double evaluate_scalar(const Eigen::Ref<const Eigen::Vector3d> &pos) const
    {
        return apex_angle_cosine_ * (pos - apex_point_).norm() - (pos - apex_point_).dot(axis_dir_);
    }

    Eigen::Vector3d evaluate_gradient(const Eigen::Ref<const Eigen::Vector3d> &pos) const
    {
        auto vec_apex = pos - apex_point_;
        auto vec_norm = vec_apex.norm();
        if (vec_norm <= kEpsilon) return Eigen::Vector3d::Zero();
        return vec_apex * apex_angle_cosine_ / vec_norm - axis_dir_;
    }

    Eigen::Vector4d evaluate_scalar_gradient(const Eigen::Ref<const Eigen::Vector3d> &pos) const
    {
        auto vec_apex = pos - apex_point_;
        auto vec_norm = vec_apex.norm();
        if (vec_norm <= kEpsilon) return Eigen::Vector4d::Zero();

        Eigen::Vector4d res{};
        res.template head<3>().array() = vec_apex.array() * apex_angle_cosine_ / vec_norm - axis_dir_.array();
        res[3]                         = apex_angle_cosine_ * vec_norm - vec_apex.dot(axis_dir_);
        return res;
    }

private:
    Eigen::Vector3d apex_point_{};
    Eigen::Vector3d axis_dir_{};
    double          apex_angle_cosine_{};
};

template <size_t Dim>
class TorusDistanceFunction;