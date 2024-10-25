#pragma once

#include "utils/eigen_alias.hpp"

class ImplicitFunction
{
public:
    template <size_t Dim>
    double evaluate_scalar(const Eigen::Ref<const Eigen::Vector<double, Dim>>& pos) const;

    template <size_t Dim>
    Eigen::Vector<double, Dim> evaluate_gradient(const Eigen::Ref<const Eigen::Vector<double, Dim>>& pos) const;

    template <size_t Dim>
    Eigen::Vector<double, Dim + 1> evaluate_scalar_gradient(const Eigen::Ref<const Eigen::Vector<double, Dim>>& pos) const;
};