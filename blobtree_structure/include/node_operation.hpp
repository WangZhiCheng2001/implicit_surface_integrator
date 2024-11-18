#pragma once

#include <type_traits>

struct node_t {
    uint64_t data[2];

    constexpr node_t() : data{0, 0} {}

    constexpr node_t(const uint64_t n1, const uint64_t n2) : data{n1, n2} {}

    template <typename T, typename = std::enable_if_t<sizeof(T) <= sizeof(uint64_t)>>
    constexpr node_t(T value)
    {
        data[0] = 0;
        data[1] = static_cast<uint64_t>(value);
    }

    template <typename T, typename = std::enable_if_t<sizeof(T) <= sizeof(uint64_t)>>
    constexpr operator T() const
    {
        return static_cast<T>(data[1]);
    }

    node_t operator>>(const uint32_t shift) const
    {
        if (shift == 0) { return *this; }

        node_t result = *this;

        if (shift >= 128) {
            result.data[0] = 0;
            result.data[1] = 0;
        } else if (shift >= 64) {
            result.data[1] = this->data[0] >> (shift - 64);
            result.data[0] = 0;
        } else {
            result.data[1] = (this->data[1] >> shift) | (this->data[0] << (64 - shift));
            result.data[0] = this->data[0] >> shift;
        }

        return result;
    }

    node_t operator<<(const uint32_t shift) const
    {
        if (shift == 0) { return *this; }

        node_t result = *this;

        if (shift >= 128) {
            result.data[0] = 0;
            result.data[1] = 0;
        } else if (shift >= 64) {
            result.data[0] = this->data[1] << (shift - 64);
            result.data[1] = 0;
        } else {
            result.data[0] = (this->data[0] << shift) | (this->data[1] >> (64 - shift));
            result.data[1] = this->data[1] << shift;
        }

        return result;
    }

    const node_t operator&(const node_t& other) const
    {
        node_t result   = *this;
        result.data[0] &= other.data[0];
        result.data[1] &= other.data[1];
        return result;
    }

    friend const node_t operator&(node_t, const uint32_t&);

    const node_t operator|(const node_t& other) const
    {
        node_t result   = *this;
        result.data[0] |= other.data[0];
        result.data[1] |= other.data[1];
        return result;
    }

    friend const node_t operator|(node_t, const uint32_t&);

    const node_t operator~() const
    {
        node_t result  = *this;
        result.data[0] = ~result.data[0];
        result.data[1] = ~result.data[1];
        return result;
    }
};

inline const node_t operator&(node_t lhs, const uint32_t& rhs)
{
    lhs.data[1] &= rhs;
    return lhs;
}

inline const node_t operator|(node_t lhs, const uint32_t& rhs)
{
    lhs.data[1] |= rhs;
    return lhs;
}

enum class eNodeLocation : uint32_t { in = 0, out = 1, edge = 2, unset = 3 };
enum class eNodeOperation : uint32_t { unionOp = 0, intersectionOp = 1, differenceOp = 2, unsetOp = 3 };

/* getter/setter for node_t */
template <typename _Tp>
struct node_proxy {
    constexpr node_proxy() = default;

    constexpr node_proxy(node_t& _data, uint32_t _offset, node_t _mask) : data(&_data), offset(_offset), mask(_mask) {}

    void reinit(node_t& _data, uint32_t _offset, node_t _mask)
    {
        data   = &_data;
        offset = _offset;
        mask   = _mask;
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
        *data            = (*data & ~_mask) | (other.data & _mask);
        return *this;
    }

    constexpr node_proxy& operator=(node_proxy&& other)
    {
        const auto _mask = mask << offset;
        *data            = (*data & ~_mask) | (std::forward(other.data) & _mask);
        return *this;
    }

    template <typename _Fp, typename = std::enable_if_t<!std::is_same_v<_Fp, node_proxy>>>
    constexpr node_proxy& operator=(_Fp&& other)
    {
        const auto _mask = mask << offset;
        if constexpr (std::is_enum_v<_Fp>) {
            const auto offset_value = static_cast<std::underlying_type_t<_Fp>>(std::forward<_Fp>(other)) << offset;
            *data                   = (*data & ~_mask) | (_mask & offset_value);
        } else if constexpr (std::is_same_v<_Fp, bool>) {
            const auto offset_value = static_cast<uint32_t>(std::forward<_Fp>(other)) << offset;
            *data                   = (*data & ~_mask) | (_mask & offset_value);
        } else {
            const auto offset_value = std::forward<_Fp>(other) << offset;
            *data                   = (*data & ~_mask) | (_mask & offset_value);
        }
        return *this;
    }

    template <typename _Fp,
              typename = std::enable_if_t<!std::is_same_v<_Fp, node_proxy> && //
                                          !std::is_enum_v<_Fp> &&             //
                                          !std::is_same_v<_Fp, bool>>>
    constexpr node_proxy& operator+=(_Fp&& other)
    {
        const auto _mask      = mask << offset;
        const _Fp  low_data   = (*data) >> offset;
        const auto low_result = std::forward<_Fp>(other) + low_data;
        *data                 = (*data & ~_mask) | (_mask & (low_result << offset));
        return *this;
    }

    constexpr operator _Tp() const { return static_cast<_Tp>(((*data) >> offset) & mask); }

protected:
    node_t*  data;
    uint32_t offset{};
    node_t   mask{};
};

template <typename _Tp>
struct const_node_proxy {
    constexpr const_node_proxy(const node_t& _data, uint32_t _offset, node_t _mask) : data(&_data), offset(_offset), mask(_mask)
    {
    }

    const_node_proxy(const const_node_proxy&) = delete;
    const_node_proxy(const_node_proxy&&)      = delete;

    constexpr operator _Tp() const { return static_cast<_Tp>(((*data) >> offset) & mask); }

protected:
    const node_t* data;
    uint32_t      offset{};
    node_t        mask{};
};

// 0 for internal node, 1 for primitive node
static constexpr inline auto node_fetch_is_primitive(node_t& node) { return node_proxy<bool>(node, 127u, 0x01u); }

// 0 for union, 1 for intersection, 2 for difference, 3 for unset
static constexpr inline auto node_fetch_operation(node_t& node) { return node_proxy<eNodeOperation>(node, 125u, 0x03u); }

// 0 for in, 1 for out, 2 for on edge, 3 for unset
static constexpr inline auto node_fetch_in_out(node_t& node) { return node_proxy<eNodeLocation>(node, 123u, 0x03u); }

// If primitive node, the index to the primitive information
static constexpr inline auto node_fetch_primitive_index(node_t& node) { return node_proxy<uint32_t>(node, 96u, 0xFFFFFFu); }

static constexpr inline auto node_is_parent_null(const node_t& node)
{
    return const_node_proxy<uint32_t>(node, 96u, 0xFFFFFFu) == 0xFFFFFFFFu;
}

// Parent node index
static constexpr inline auto node_fetch_parent_index(node_t& node) { return node_proxy<uint32_t>(node, 64u, 0xFFFFFFFFu); }

// Left child node index
static constexpr inline auto node_fetch_left_child_index(node_t& node) { return node_proxy<uint32_t>(node, 32u, 0xFFFFFFFFu); }

static constexpr inline auto node_is_left_child_null(const node_t& node)
{
    return const_node_proxy<uint32_t>(node, 32u, 0xFFFFFFFFu) == 0xFFFFFFFFu;
}

// Right child node index
static constexpr inline auto node_fetch_right_child_index(node_t& node) { return node_proxy<uint32_t>(node, 0u, 0xFFFFFFFFu); }

static constexpr inline auto node_is_right_child_null(const node_t& node)
{
    return const_node_proxy<uint32_t>(node, 0u, 0xFFFFFFFFu) == 0xFFFFFFFFu;
}