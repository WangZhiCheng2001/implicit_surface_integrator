#pragma once

#include "utils/eigen_alias.hpp"

template <typename Scalar, size_t Dim>
class ImplicitFunction
{
public:
    Scalar                         evaluate_scalar(const Eigen::Ref<const Eigen::Vector<Scalar, Dim>>& pos) const;
    Eigen::Vector<Scalar, Dim>     evaluate_gradient(const Eigen::Ref<const Eigen::Vector<Scalar, Dim>>& pos) const;
    Eigen::Vector<Scalar, Dim + 1> evaluate_scalar_gradient(const Eigen::Ref<const Eigen::Vector<Scalar, Dim>>& pos) const;
};