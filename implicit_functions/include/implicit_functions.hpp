#pragma once

#include <variant>

#include <primitive_functions.hpp>

template <typename Scalar, size_t Dim>
using implicit_function_t = std::variant<ConstantFunction<Scalar, Dim>,
                                         PlaneDistanceFunction<Scalar, Dim>,
                                         CylinderDistanceFunction<Scalar, Dim>,
                                         SphereDistanceFunction<Scalar, Dim>,
                                         ConeDistanceFunction<Scalar, Dim>>;