#pragma once

#include <bitset>

#include <utils/eigen_alias.hpp>

// ======================================================================
// AABB
// ======================================================================

struct aabb_t {
    Eigen::Vector3d min{std::numeric_limits<double>::max(),
                        std::numeric_limits<double>::max(),
                        std::numeric_limits<double>::max()};
    Eigen::Vector3d max{std::numeric_limits<double>::min(),
                        std::numeric_limits<double>::min(),
                        std::numeric_limits<double>::min()};

    void extend(const Eigen::Vector3d& point)
    {
        min = min.cwiseMin(point);
        max = max.cwiseMax(point);
    }

    void extend(const aabb_t& aabb)
    {
        min = min.cwiseMin(aabb.min);
        max = max.cwiseMax(aabb.max);
    }

    void offset(const Eigen::Vector3d& offset)
    {
        min = min + offset;
        max = max + offset;
    }

    bool contains(const Eigen::Vector3d& point) const
    {
        return (point.array() >= min.array()).all() && (point.array() <= max.array()).all();
    }
};

// ======================================================================
// Node
// ======================================================================

using node_t = std::bitset<128>;

static const node_t standard_new_node = [] {
    node_t node{};
    return node.flip();
}();

enum class eNodeLocation : uint32_t { in = 0, out = 1, edge = 2, unset = 3 };
enum class eNodeOperation : uint32_t { unionOp = 0, intersectionOp = 1, differenceOp = 2, unsetOp = 3 };

/* getter/setter for node_t */
template <typename _Tp>
struct node_proxy {
    constexpr node_proxy() = default;

    constexpr node_proxy(node_t& _data, uint32_t _offset, uint32_t _mask_bits)
        : data(_data), offset(_offset), mask((1ull << _mask_bits) - 1ull)
    {
    }

    void reinit(node_t& _data, uint32_t _offset, uint32_t _mask_bits)
    {
        data   = _data;
        offset = _offset;
        mask   = (1ull << _mask_bits) - 1ull;
    }

    void reinit(node_proxy&& other)
    {
        data   = std::move(other.data);
        offset = std::move(other.offset);
        mask   = std::move(other.mask);
    }

    node_proxy(const node_proxy&) = delete;
    node_proxy(node_proxy&&)      = delete;

    constexpr node_proxy& operator=(const node_proxy& other)
    {
        const auto _mask = mask << offset;
        data             = (data & ~_mask) | (other.data & _mask);
        return *this;
    }

    constexpr node_proxy& operator=(node_proxy&& other)
    {
        const auto _mask = mask << offset;
        data             = (data & ~_mask) | (std::forward(other.data) & _mask);
        return *this;
    }

    template <typename _Fp, typename = std::enable_if_t<!std::is_same_v<_Fp, node_proxy>>>
    constexpr node_proxy& operator=(_Fp&& other)
    {
        const auto _mask        = mask << offset;
        const auto offset_value = node_t{static_cast<uint64_t>(std::forward<_Fp>(other))} << offset;
        data                    = (data & ~_mask) | (offset_value & _mask);
        return *this;
    }

    template <typename _Fp,
              typename = std::enable_if_t<!std::is_same_v<_Fp, node_proxy> && //
                                          !std::is_enum_v<_Fp> &&             //
                                          !std::is_same_v<_Fp, bool>>>
    constexpr node_proxy& operator+=(_Fp&& other)
    {
        const auto _mask      = mask << offset;
        const auto low_data   = (data >> offset).to_ullong();
        const auto low_result = static_cast<uint64_t>(std::forward<_Fp>(other)) + low_data;
        data                  = (data & ~_mask) | ((node_t{low_result} << offset) & _mask);
        return *this;
    }

    constexpr operator _Tp() const { return static_cast<_Tp>((((data) >> offset) & mask).to_ulong()); }

protected:
    node_t&  data;
    uint32_t offset{};
    node_t   mask{};
};

template <typename _Tp>
struct const_node_proxy {
    constexpr const_node_proxy(const node_t& _data, uint32_t _offset, uint32_t _mask_bits)
        : data(_data), offset(_offset), mask((1ull << _mask_bits) - 1ull)
    {
    }

    const_node_proxy(const const_node_proxy&) = delete;
    const_node_proxy(const_node_proxy&&)      = delete;

    constexpr operator _Tp() const { return static_cast<_Tp>((((data) >> offset) & mask).to_ulong()); }

protected:
    const node_t& data;
    uint32_t      offset{};
    node_t        mask{};
};

// 0 for internal node, 1 for primitive node
static constexpr inline auto node_fetch_is_primitive(node_t& node) { return node_proxy<bool>(node, 127u, 1); }

// 0 for union, 1 for intersection, 2 for difference, 3 for unset
static constexpr inline auto node_fetch_operation(node_t& node) { return node_proxy<eNodeOperation>(node, 125u, 2); }

// 0 for in, 1 for out, 2 for on edge, 3 for unset
static constexpr inline auto node_fetch_in_out(node_t& node) { return node_proxy<eNodeLocation>(node, 123u, 2); }

// If primitive node, the index to the primitive information
static constexpr inline auto node_fetch_primitive_index(node_t& node) { return node_proxy<uint32_t>(node, 96u, 24); }

// Parent node index
static constexpr inline auto node_fetch_parent_index(node_t& node) { return node_proxy<uint32_t>(node, 64u, 32); }

static constexpr inline auto node_is_parent_null(const node_t& node)
{
    return const_node_proxy<uint32_t>(node, 64u, 32) == 0xFFFFFFFFu;
}

// Left child node index
static constexpr inline auto node_fetch_left_child_index(node_t& node) { return node_proxy<uint32_t>(node, 32u, 32); }

static constexpr inline auto node_is_left_child_null(const node_t& node)
{
    return const_node_proxy<uint32_t>(node, 32u, 32) == 0xFFFFFFFFu;
}

// Right child node index
static constexpr inline auto node_fetch_right_child_index(node_t& node) { return node_proxy<uint32_t>(node, 0u, 32); }

static constexpr inline auto node_is_right_child_null(const node_t& node)
{
    return const_node_proxy<uint32_t>(node, 0u, 32) == 0xFFFFFFFFu;
}