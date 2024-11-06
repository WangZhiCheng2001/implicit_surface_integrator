#include <stack>

#include "internal_api.hpp"
#include "primitive_descriptor.h"

static constexpr auto tree_vector_length = 65535;

/* internal global variables for blobtree */
struct blobtree_t {
    std::vector<node_t, tbb::tbb_allocator<node_t>> nodes{};
};

std::vector<blobtree_t, tbb::tbb_allocator<blobtree_t>>                  structures{};
std::vector<primitive_node_t, tbb::tbb_allocator<primitive_node_t>>      primitives{};
std::stack<uint32_t, std::deque<uint32_t, tbb::tbb_allocator<uint32_t>>> free_structure_list{};

/* getter/setter for node_t */
template <typename _Tp>
struct node_proxy {
    constexpr node_proxy(node_t& _data, uint32_t _offset, uint32_t _mask) : data(&_data), offset(_offset), mask(_mask) {}

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

    template <typename _Fp>
    constexpr node_proxy& operator=(_Fp&& other)
    {
        const auto _mask = mask << offset;
        *data            = (*data & ~_mask) | (std::forward<_Fp>(other) & _mask);
        return *this;
    }

    constexpr operator _Tp() const { return static_cast<_Tp>(((*data) >> offset) & mask); }

protected:
    node_t*  data;
    uint32_t offset{};
    uint32_t mask{};
};

// 0 for null pointer, 1 for non-null nodes
static constexpr inline auto node_fetch_is_non_null(node_t& node) { return node_proxy<bool>(node, 31u, 0x01u); }

// 0 for internal node, 1 for primitive node
static constexpr inline auto node_fetch_is_primitive(node_t& node) { return node_proxy<bool>(node, 30u, 0x01u); }

// 0 for union, 1 for intersection, 2 for difference, 3 for unset
static constexpr inline auto node_fetch_operation(node_t& node) { return node_proxy<uint32_t>(node, 28u, 0x03u); }

// 0 for no cross, 1 for cross to parent, 2 for cross left child, 3 for cross right child
static constexpr inline auto node_fetch_fetch_cross(node_t& node) { return node_proxy<bool>(node, 26u, 0x03u); }

// If primitive node, the index to the primitive information, if cross node, the index to the cross node
static constexpr inline auto node_fetch_primitive_index(node_t& node) { return node_proxy<bool>(node, 10u, 0xFFFFu); }

// use in cross node
static constexpr inline auto node_fetch_main_index(node_t& node) { return node_proxy<bool>(node, 2u, 0xFFu); }

/* basic functionalities */
// primitives
BPE_API std::vector<primitive_node_t, tbb::tbb_allocator<primitive_node_t>>& get_primitives() { return primitives; }

void shrink_primitives() { primitives.shrink_to_fit(); }

// sub tree
uint32_t create_new_sub_blobtree()
{
    auto& tree = structures.emplace_back();
    tree.nodes.reserve(tree_vector_length);
    primitives.reserve(primitives.capacity() + ((tree_vector_length + 1) >> 1));
    return static_cast<uint32_t>(structures.size() - 1);
}

void free_sub_blobtree(uint32_t index)
{
    // 这里尽量打标记，延迟修改和删除
    free_structure_list.push(index);
}

// node insertion
static constexpr auto standard_new_node = 0xF0000000u; // i.e. non_null, primitive, but invalid operation
                                                       // others are all zero-valued

virtual_node_t push_primitive_node(primitive_node_t&& primitive_node)
{
    primitives.emplace_back(primitive_node);
    auto& node                       = structures[0].nodes.emplace_back(standard_new_node);
    node_fetch_primitive_index(node) = static_cast<uint32_t>(primitives.size() - 1);

    return {0, static_cast<uint32_t>(primitives.size() - 1)};
}

BPE_API virtual_node_t blobtree_new_virtual_node(const constant_descriptor_t& desc)
{
    primitive_node_t node{PRIMITIVE_TYPE_CONSTANT, malloc(sizeof(constant_descriptor_t))};
    std::move(&desc, &desc + 1, node.desc);
    return push_primitive_node(std::move(node));
}

BPE_API virtual_node_t blobtree_new_virtual_node(const plane_descriptor_t& desc)
{
    primitive_node_t node{PRIMITIVE_TYPE_PLANE, malloc(sizeof(plane_descriptor_t))};
    std::move(&desc, &desc + 1, node.desc);
    return push_primitive_node(std::move(node));
}

BPE_API virtual_node_t blobtree_new_virtual_node(const sphere_descriptor_t& desc)
{
    primitive_node_t node{PRIMITIVE_TYPE_SPHERE, malloc(sizeof(sphere_descriptor_t))};
    std::move(&desc, &desc + 1, node.desc);
    return push_primitive_node(std::move(node));
}

BPE_API virtual_node_t blobtree_new_virtual_node(const cylinder_descriptor_t& desc)
{
    primitive_node_t node{PRIMITIVE_TYPE_CYLINDER, malloc(sizeof(cylinder_descriptor_t))};
    std::move(&desc, &desc + 1, node.desc);
    return push_primitive_node(std::move(node));
}

BPE_API virtual_node_t blobtree_new_virtual_node(const cone_descriptor_t& desc)
{
    primitive_node_t node{PRIMITIVE_TYPE_CONE, malloc(sizeof(cone_descriptor_t))};
    std::move(&desc, &desc + 1, node.desc);
    return push_primitive_node(std::move(node));
}

BPE_API virtual_node_t blobtree_new_virtual_node(const box_descriptor_t& desc)
{
    primitive_node_t node{PRIMITIVE_TYPE_BOX, malloc(sizeof(box_descriptor_t))};
    std::move(&desc, &desc + 1, node.desc);
    return push_primitive_node(std::move(node));
}

BPE_API virtual_node_t blobtree_new_virtual_node(const mesh_descriptor_t& desc)
{
    primitive_node_t node{PRIMITIVE_TYPE_MESH, malloc(sizeof(mesh_descriptor_t))};
    std::move(&desc, &desc + 1, node.desc);
    return push_primitive_node(std::move(node));
}

BPE_API virtual_node_t blobtree_new_virtual_node(const extrude_descriptor_t& desc)
{
    primitive_node_t node{PRIMITIVE_TYPE_EXTRUDE, malloc(sizeof(extrude_descriptor_t))};
    std::move(&desc, &desc + 1, node.desc);
    return push_primitive_node(std::move(node));
}