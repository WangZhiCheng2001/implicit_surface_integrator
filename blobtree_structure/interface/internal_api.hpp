#pragma once

#include <tbb/tbb_allocator.h>

#include <macros.h>
#include "blobtree.h"
#include <utils/eigen_alias.hpp>

BPE_API double evaluate(const constant_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point);
BPE_API double evaluate(const plane_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point);
BPE_API double evaluate(const sphere_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point);
BPE_API double evaluate(const cylinder_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point);
BPE_API double evaluate(const cone_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point);
BPE_API double evaluate(const box_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point);
BPE_API double evaluate(const mesh_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point);
BPE_API double evaluate(const extrude_descriptor_t& desc, const Eigen::Ref<const Eigen::Vector3d>& point);

BPE_API virtual_node_t blobtree_new_virtual_node(const constant_descriptor_t& desc);
BPE_API virtual_node_t blobtree_new_virtual_node(const plane_descriptor_t& desc);
BPE_API virtual_node_t blobtree_new_virtual_node(const sphere_descriptor_t& desc);
BPE_API virtual_node_t blobtree_new_virtual_node(const cylinder_descriptor_t& desc);
BPE_API virtual_node_t blobtree_new_virtual_node(const cone_descriptor_t& desc);
BPE_API virtual_node_t blobtree_new_virtual_node(const box_descriptor_t& desc);
BPE_API virtual_node_t blobtree_new_virtual_node(const mesh_descriptor_t& desc);
BPE_API virtual_node_t blobtree_new_virtual_node(const extrude_descriptor_t& desc);

BPE_API std::vector<primitive_node_t, tbb::tbb_allocator<primitive_node_t>>& get_primitives();