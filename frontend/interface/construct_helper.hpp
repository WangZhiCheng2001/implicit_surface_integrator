#pragma once

#include <type_traits>

#include "blobtree.h"
#include "io.h"

namespace detail
{
template <typename T>
struct primitive_type_info;

template <>
struct primitive_type_info<constant_descriptor_t> {
    static constexpr primitive_type type = PRIMITIVE_TYPE_CONSTANT;
};

template <>
struct primitive_type_info<plane_descriptor_t> {
    static constexpr primitive_type type = PRIMITIVE_TYPE_PLANE;
};

template <>
struct primitive_type_info<sphere_descriptor_t> {
    static constexpr primitive_type type = PRIMITIVE_TYPE_SPHERE;
};

template <>
struct primitive_type_info<cylinder_descriptor_t> {
    static constexpr primitive_type type = PRIMITIVE_TYPE_CYLINDER;
};

template <>
struct primitive_type_info<cone_descriptor_t> {
    static constexpr primitive_type type = PRIMITIVE_TYPE_CONE;
};

template <>
struct primitive_type_info<box_descriptor_t> {
    static constexpr primitive_type type = PRIMITIVE_TYPE_BOX;
};

template <>
struct primitive_type_info<mesh_descriptor_t> {
    static constexpr primitive_type type = PRIMITIVE_TYPE_MESH;
};

template <>
struct primitive_type_info<extrude_descriptor_t> {
    static constexpr primitive_type type = PRIMITIVE_TYPE_EXTRUDE;
};

template <typename T>
static constexpr bool is_primitive_descriptor_t = std::is_same_v<const primitive_type, decltype(primitive_type_info<T>::type)>;
} // namespace detail

template <typename T,
          typename = std::enable_if_t<detail::is_primitive_descriptor_t<std::remove_cv_t<std::remove_reference_t<T>>>>>
static inline virtual_node_t make_primitive_node(T&& descriptor)
{
    using value_type = std::remove_cv_t<std::remove_reference_t<T>>;
    if constexpr (std::is_rvalue_reference_v<T&&>)
        return blobtree_new_node_by_move(movable_descriptor_t{const_cast<value_type*>(&descriptor)},
                                         detail::primitive_type_info<value_type>::type);
    else
        return blobtree_new_node_by_copy(copyable_descriptor_t{const_cast<value_type*>(&descriptor)},
                                         detail::primitive_type_info<value_type>::type);
}

template <typename T, typename = std::enable_if_t<detail::is_primitive_descriptor_t<T>>>
static inline virtual_node_t make_primitive_node_by_copy(const T& descriptor)
{
    return make_primitive_node(descriptor);
}

template <typename T, typename = std::enable_if_t<detail::is_primitive_descriptor_t<T>>>
static inline virtual_node_t make_primitive_node_by_move(const T& descriptor)
{
    return make_primitive_node(std::move(descriptor));
}

template <typename T,
          typename = std::enable_if_t<detail::is_primitive_descriptor_t<std::remove_cv_t<std::remove_reference_t<T>>>>>
static inline bool replace_primitive_node(const virtual_node_t& node, T&& descriptor)
{
    using value_type = std::remove_cv_t<std::remove_reference_t<T>>;
    if constexpr (std::is_rvalue_reference_v<T&&>)
        return virtual_node_replace_primitive_by_move(node,
                                                      movable_descriptor_t{const_cast<value_type*>(&descriptor)},
                                                      detail::primitive_type_info<value_type>::type);
    else
        return virtual_node_replace_primitive_by_copy(node,
                                                      copyable_descriptor_t{const_cast<value_type*>(&descriptor)},
                                                      detail::primitive_type_info<value_type>::type);
}

template <typename T, typename = std::enable_if_t<detail::is_primitive_descriptor_t<T>>>
static inline bool replace_primitive_node_by_copy(const virtual_node_t& node, const T& descriptor)
{
    return replace_primitive_node(node, descriptor);
}

template <typename T, typename = std::enable_if_t<detail::is_primitive_descriptor_t<T>>>
static inline virtual_node_t replace_primitive_node_by_move(const virtual_node_t& node, const T& descriptor)
{
    return replace_primitive_node(node, std::move(descriptor));
}