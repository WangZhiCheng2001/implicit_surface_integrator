#pragma once

#include <macros.h>
#include <utils/eigen_alias.hpp>

#include <primitive_descriptor.h>

PE_API double evaluate(const constant_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point);
PE_API double evaluate(const plane_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point);
PE_API double evaluate(const sphere_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point);
PE_API double evaluate(const cylinder_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point);
PE_API double evaluate(const cone_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point);
PE_API double evaluate(const box_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point);
PE_API double evaluate(const mesh_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point);
PE_API double evaluate(const extrude_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point);
