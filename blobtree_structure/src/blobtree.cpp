#include <cassert>
#include <stack>
#include <vector>

#include "internal_api.hpp"

/* internal global variables for blobtree */
typedef struct _node_t {
    uint64_t data[2];

    constexpr _node_t() : data{0, 0} {}

    constexpr _node_t(const uint64_t n1, const uint64_t n2) : data{n1, n2} {}

    template <typename T, typename = std::enable_if_t<sizeof(T) <= sizeof(uint64_t)>>
    constexpr _node_t(T value)
    {
        data[0] = 0;
        data[1] = static_cast<uint64_t>(value);
    }

    template <typename T, typename = std::enable_if_t<sizeof(T) <= sizeof(uint64_t)>>
    constexpr operator T() const
    {
        return static_cast<T>(data[1]);
    }

    _node_t operator>>(const uint32_t shift) const
    {
        if (shift == 0) { return *this; }

        _node_t result = *this;

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

    _node_t operator<<(const uint32_t shift) const
    {
        if (shift == 0) { return *this; }

        _node_t result = *this;

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

    const _node_t operator&(const _node_t& other) const
    {
        _node_t result  = *this;
        result.data[0] &= other.data[0];
        result.data[1] &= other.data[1];
        return result;
    }

    const _node_t operator&(const uint32_t other) const
    {
        _node_t result  = *this;
        result.data[1] &= other;
        return result;
    }

    const _node_t operator|(const _node_t& other) const
    {
        _node_t result  = *this;
        result.data[0] |= other.data[0];
        result.data[1] |= other.data[1];
        return result;
    }

    const _node_t operator|(const uint32_t other) const
    {
        _node_t result  = *this;
        result.data[1] |= other;
        return result;
    }

    const _node_t operator~() const
    {
        _node_t result = *this;
        result.data[0] = ~result.data[0];
        result.data[1] = ~result.data[1];
        return result;
    }

} node_t;

const raw_vector3d_t operator+(const raw_vector3d_t& point1, const raw_vector3d_t& point2)
{
    raw_vector3d_t result;
    result.x = point1.x + point2.x;
    result.y = point1.y + point2.y;
    result.z = point1.z + point2.z;
    return result;
}

const raw_vector3d_t operator-(const raw_vector3d_t& point1, const raw_vector3d_t& point2)
{
    raw_vector3d_t result;
    result.x = point1.x - point2.x;
    result.y = point1.y - point2.y;
    result.z = point1.z - point2.z;
    return result;
}

const raw_vector3d_t operator+(const raw_vector3d_t& point, const double value)
{
    raw_vector3d_t result;
    result.x = point.x + value;
    result.y = point.y + value;
    result.z = point.z + value;
    return result;
}

const raw_vector3d_t operator-(const raw_vector3d_t& point, const double value)
{
    raw_vector3d_t result;
    result.x = point.x - value;
    result.y = point.y - value;
    result.z = point.z - value;
    return result;
}

typedef struct _aabb_t {
    raw_vector3d_t min;
    raw_vector3d_t max;

    _aabb_t()
    {
        min.x = std::numeric_limits<double>::max();
        min.y = std::numeric_limits<double>::max();
        min.z = std::numeric_limits<double>::max();
        max.x = std::numeric_limits<double>::min();
        max.y = std::numeric_limits<double>::min();
        max.z = std::numeric_limits<double>::min();
    }

    _aabb_t(const raw_vector3d_t& min, const raw_vector3d_t& max) : min(min), max(max) {}

    void extend(const raw_vector3d_t& point)
    {
        min.x = std::min(min.x, point.x);
        min.y = std::min(min.y, point.y);
        min.z = std::min(min.z, point.z);
        max.x = std::max(max.x, point.x);
        max.y = std::max(max.y, point.y);
        max.z = std::max(max.z, point.z);
    }

    void extend(const _aabb_t& aabb)
    {
        min.x = std::min(min.x, aabb.min.x);
        min.y = std::min(min.y, aabb.min.y);
        min.z = std::min(min.z, aabb.min.z);
        max.x = std::max(max.x, aabb.max.x);
        max.y = std::max(max.y, aabb.max.y);
        max.z = std::max(max.z, aabb.max.z);
    }

    void offset(const raw_vector3d_t& offset)
    {
        min = min + offset;
        max = max + offset;
    }

} aabb_t;

struct blobtree_t {
    std::vector<node_t, tbb::tbb_allocator<node_t>>     nodes{};
    std::vector<uint32_t, tbb::tbb_allocator<uint32_t>> leaf_index{};
};

std::vector<blobtree_t, tbb::tbb_allocator<blobtree_t>>                  structures{};
std::vector<aabb_t, tbb::tbb_allocator<aabb_t>>                          aabbs{};
std::vector<primitive_node_t, tbb::tbb_allocator<primitive_node_t>>      primitives{};
std::stack<uint32_t, std::deque<uint32_t, tbb::tbb_allocator<uint32_t>>> free_structure_list{};

/* getter/setter for node_t */
template <typename _Tp>
struct node_proxy {
    constexpr node_proxy(node_t& _data, uint32_t _offset, node_t _mask) : data(&_data), offset(_offset), mask(_mask) {}

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
    node_t   mask{};
};

#define NODE_LOCATION_IN    0
#define NODE_LOCATION_OUT   1
#define NODE_LOCATION_EDGE  2
#define NODE_LOCATION_UNSET 3

#define OP_UNION        0
#define OP_INTERSECTION 1
#define OP_DIFFERENCE   2
#define OP_UNSET        3

// 0 for internal node, 1 for primitive node
static constexpr inline auto node_fetch_is_primitive(node_t& node) { return node_proxy<bool>(node, 127u, 0x01u); }

// 0 for union, 1 for intersection, 2 for difference, 3 for unset
static constexpr inline auto node_fetch_operation(node_t& node) { return node_proxy<uint32_t>(node, 125u, 0x03u); }

// 0 for in, 1 for out, 2 for on edge, 3 for unset
static constexpr inline auto node_fetch_in_out(node_t& node) { return node_proxy<uint32_t>(node, 123u, 0x03u); }

// If primitive node, the index to the primitive information
static constexpr inline auto node_fetch_primitive_index(node_t& node) { return node_proxy<uint32_t>(node, 96u, 0xFFFFFFu); }

// Parent node index
static constexpr inline auto node_fetch_parent_index(node_t& node) { return node_proxy<uint32_t>(node, 64u, 0xFFFFFFFFu); }

// Left child node index
static constexpr inline auto node_fetch_left_child_index(node_t& node) { return node_proxy<uint32_t>(node, 32u, 0xFFFFFFFFu); }

// Right child node index
static constexpr inline auto node_fetch_right_child_index(node_t& node) { return node_proxy<uint32_t>(node, 0u, 0xFFFFFFFFu); }

/* basic functionalities */

BPE_API std::vector<primitive_node_t, tbb::tbb_allocator<primitive_node_t>>& get_primitives() noexcept { return primitives; }

void shrink_primitives() { primitives.shrink_to_fit(); }

bool is_primitive_node(node_t& node) { return node_fetch_is_primitive(node) == 1; }

bool is_parent_null(node_t& node) { return node_fetch_parent_index(node) == 0xFFFFFFFFu; }

bool is_left_null(node_t& node) { return node_fetch_left_child_index(node) == 0xFFFFFFFFu; }

bool is_right_null(node_t& node) { return node_fetch_right_child_index(node) == 0xFFFFFFFFu; }

void set_is_primitive(node_t& node, const bool flag)
{
    node_t temp;
    temp.data[0]                  = flag;
    temp.data[0]                  = temp.data[0] << 63;
    node_fetch_is_primitive(node) = temp;
}

void set_operation(node_t& node, const uint32_t op)
{
    node_t temp;
    temp.data[0]               = op;
    temp.data[0]               = temp.data[0] << 61;
    node_fetch_operation(node) = temp;
}

void set_in_out(node_t& node, const uint32_t in_out)
{
    node_t temp;
    temp.data[0]            = in_out;
    temp.data[0]            = temp.data[0] << 59;
    node_fetch_in_out(node) = temp;
}

void set_primitive_index(node_t& node, const uint32_t index)
{
    node_t temp;
    temp.data[0]                     = index;
    temp.data[0]                     = temp.data[0] << 32;
    node_fetch_primitive_index(node) = temp;
}

void set_parent_index(node_t& node, const uint32_t index)
{
    node_t temp;
    temp.data[0]                  = index;
    node_fetch_parent_index(node) = temp;
}

void set_left_child_index(node_t& node, const uint32_t index)
{
    node_t temp;
    temp.data[1]                      = index;
    temp.data[1]                      = temp.data[1] << 32;
    node_fetch_left_child_index(node) = temp;
}

void set_right_child_index(node_t& node, const uint32_t index)
{
    node_t temp;
    temp.data[1]                       = index;
    node_fetch_right_child_index(node) = temp;
}

virtual_node_t copy(virtual_node_t old_node, virtual_node_t new_node)
{
    assert(old_node.main_index != new_node.main_index);

    // Copy a tree and its subtrees to a temporary tree
    auto temp = structures[old_node.main_index];

    // Update all index
    int size = structures[new_node.main_index].nodes.size();
    for (int i = 0; i < temp.nodes.size(); i++) {
        if (!is_parent_null(temp.nodes[i])) { set_parent_index(temp.nodes[i], node_fetch_parent_index(temp.nodes[i]) + size); }
        if (!is_left_null(temp.nodes[i])) {
            set_left_child_index(temp.nodes[i], node_fetch_left_child_index(temp.nodes[i]) + size);
        }
        if (!is_right_null(temp.nodes[i])) {
            set_right_child_index(temp.nodes[i], node_fetch_right_child_index(temp.nodes[i]) + size);
        }
    }
    for (int i = 0; i < temp.leaf_index.size(); i++) { temp.leaf_index[i] += size; }

    // Copy the updated index tree to the array at the new location
    structures[new_node.main_index].nodes.insert(structures[new_node.main_index].nodes.end(),
                                                 temp.nodes.begin(),
                                                 temp.nodes.end());
    structures[new_node.main_index].leaf_index.insert(structures[new_node.main_index].leaf_index.end(),
                                                      temp.leaf_index.begin(),
                                                      temp.leaf_index.end());

    return virtual_node_t{new_node.main_index, old_node.inner_index + size};
}

void offset_primitive(primitive_node_t& node, const raw_vector3d_t& offset)
{
    auto offset_point = [](raw_vector3d_t* point, const raw_vector3d_t& offset) {
        point->x += offset.x;
        point->y += offset.y;
        point->z += offset.z;
    };

    auto type = node.type;
    switch (type) {
        case PRIMITIVE_TYPE_CONSTANT: {
            break;
        }
        case PRIMITIVE_TYPE_PLANE: {
            auto desc = static_cast<plane_descriptor_t*>(node.desc);
            offset_point(&desc->point, offset);
            break;
        }
        case PRIMITIVE_TYPE_SPHERE: {
            auto desc = static_cast<sphere_descriptor_t*>(node.desc);
            offset_point(&desc->center, offset);
            break;
        }
        case PRIMITIVE_TYPE_CYLINDER: {
            auto desc = static_cast<cylinder_descriptor_t*>(node.desc);
            offset_point(&desc->bottom_origion, offset);
            break;
        }
        case PRIMITIVE_TYPE_CONE: {
            auto desc = static_cast<cone_descriptor_t*>(node.desc);
            offset_point(&desc->top_point, offset);
            offset_point(&desc->bottom_point, offset);
            break;
        }
        case PRIMITIVE_TYPE_BOX: {
            auto desc = static_cast<box_descriptor_t*>(node.desc);
            offset_point(&desc->center, offset);

            break;
        }
        case PRIMITIVE_TYPE_MESH: {
            auto desc = static_cast<mesh_descriptor_t*>(node.desc);
            for (int i = 0; i < desc->point_number; i++) { offset_point(&desc->points[i], offset); }
            break;
        }
        case PRIMITIVE_TYPE_EXTRUDE: {
            auto desc = static_cast<extrude_descriptor_t*>(node.desc);
            for (int i = 0; i < desc->edges_number; i++) { offset_point(&desc->points[i], offset); }
            break;
        }
        default: {
            break;
        }
    }
}

void free_sub_blobtree(uint32_t index)
{
    // 这里尽量打标记，延迟修改和删除
    free_structure_list.push(index);
}

bool upward_propagation(blobtree_t& tree, const int leaf_node_index, const int root_index)
{
    int now_index = leaf_node_index;
    now_index     = node_fetch_parent_index(tree.nodes[now_index]);

    while (true) {
        auto& node        = tree.nodes[now_index];
        auto& left_child  = tree.nodes[node_fetch_left_child_index(node)];
        auto& right_child = tree.nodes[node_fetch_right_child_index(node)];

        if (node_fetch_in_out(node) != NODE_LOCATION_UNSET) { return false; }

        switch (node_fetch_operation(node)) {
            case OP_UNION: {
                if (node_fetch_in_out(left_child) == NODE_LOCATION_IN || node_fetch_in_out(right_child) == NODE_LOCATION_IN) {
                    set_in_out(node, NODE_LOCATION_IN);
                } else if (node_fetch_in_out(left_child) == NODE_LOCATION_OUT
                           && node_fetch_in_out(right_child) == NODE_LOCATION_OUT) {
                    set_in_out(node, NODE_LOCATION_OUT);
                } else {
                    return false;
                }
                break;
            }
            case OP_INTERSECTION: {
                if (node_fetch_in_out(left_child) == NODE_LOCATION_IN && node_fetch_in_out(right_child) == NODE_LOCATION_IN) {
                    set_in_out(node, NODE_LOCATION_IN);
                } else if (node_fetch_in_out(left_child) == NODE_LOCATION_OUT
                           || node_fetch_in_out(right_child) == NODE_LOCATION_OUT) {
                    set_in_out(node, NODE_LOCATION_OUT);
                } else {
                    return false;
                }
                break;
            }
            case OP_DIFFERENCE: {
                if (node_fetch_in_out(left_child) == NODE_LOCATION_IN && node_fetch_in_out(right_child) == NODE_LOCATION_OUT) {
                    set_in_out(node, NODE_LOCATION_IN);
                }
                if (node_fetch_in_out(left_child) == NODE_LOCATION_OUT || node_fetch_in_out(right_child) == NODE_LOCATION_IN) {
                    set_in_out(node, NODE_LOCATION_OUT);
                } else {
                    return false;
                }
                break;
            }
            default: {
                return false;
                break;
            }
        }

        if (now_index == root_index) { return true; }

        now_index = node_fetch_parent_index(node);
    }
}

int evaluate(const virtual_node_t& node, const raw_vector3d_t& point)
{
    auto temp = structures[node.main_index];

    auto& leaf_index = temp.leaf_index;
    for (int i = 0; i < leaf_index.size(); i++) {
        auto sdf = evaluate(primitives[leaf_index[i]], point);
        if (sdf <= 0.0) {
            set_in_out(temp.nodes[leaf_index[i]], NODE_LOCATION_IN);
        } else {
            set_in_out(temp.nodes[leaf_index[i]], NODE_LOCATION_OUT);
        }

        if (upward_propagation(temp, leaf_index[i], node.inner_index)) { break; }
    }

    return node_fetch_in_out(temp.nodes[node.inner_index]);
}

aabb_t get_aabb(const virtual_node_t& node)
{
    auto&  leaf_index = structures[node.main_index].leaf_index;
    aabb_t result{};
    for (auto& index : leaf_index) {
        auto& type = primitives[index].type;
        if (type != PRIMITIVE_TYPE_CONSTANT && type != PRIMITIVE_TYPE_PLANE) { result.extend(aabbs[index]); }
    }
    return result;
}

uint32_t get_closest_common_parent(const std::vector<bool>& mask, const int main_index)
{
    // Copy tree structure
    auto tree = structures[main_index];

    // Count how many geometries are queried
    int count = 0;
    for (int i = 0; i < mask.size(); i++) {
        if (mask[i] == 1) { count++; }
    }

    uint32_t result = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < mask.size(); i++) {
        if (mask[i] == 0) { continue; }

        for (auto& iter : tree.leaf_index) {
            // Find the location of the current query geometry in the tree
            if (node_fetch_primitive_index(tree.nodes[iter]) != i) { continue; }

            // Traverse from bottom to top and increase the count of all nodes by 1
            uint32_t now = iter;
            while (true) {
                now = node_fetch_parent_index(tree.nodes[now]);

                // now is root
                if (now == 0xFFFFFFFFu) { break; }

                // Use the primitive index of the internal node to count
                if (node_fetch_primitive_index(tree.nodes[now]) == 0xFFFFFFu) {
                    set_primitive_index(tree.nodes[now], 1);
                } else {
                    set_primitive_index(tree.nodes[now], node_fetch_primitive_index(tree.nodes[now]) + 1);
                }

                if (node_fetch_primitive_index(tree.nodes[now]) == count) {
                    result = now;
                    break;
                }
            }

            break;
        }
    }

    return result;
}

/* Geometry Generation */

static constexpr node_t standard_new_node = {(uint64_t)0xFFFFFFFFFFFFFFFFu, (uint64_t)0xFFFFFFFFFFFFFFFFu};

virtual_node_t push_primitive_node(primitive_node_t&& primitive_node, const aabb_t&& aabb)
{
    aabbs.emplace_back(aabb);
    primitives.emplace_back(primitive_node);

    node_t node = standard_new_node;
    set_primitive_index(node, static_cast<uint32_t>(primitives.size() - 1));

    blobtree_t tree;
    tree.nodes.emplace_back(node);
    tree.leaf_index.push_back(0);

    structures.push_back(tree);

    return virtual_node_t{static_cast<uint32_t>(structures.size() - 1), 0};
}

BPE_API virtual_node_t blobtree_new_virtual_node(const constant_descriptor_t& desc)
{
    primitive_node_t node{PRIMITIVE_TYPE_CONSTANT, malloc(sizeof(constant_descriptor_t))};
    *((constant_descriptor_t*)node.desc) = std::move(desc);
    aabb_t aabb{};
    return push_primitive_node(std::move(node), std::move(aabb));
}

BPE_API virtual_node_t blobtree_new_virtual_node(const plane_descriptor_t& desc)
{
    primitive_node_t node{PRIMITIVE_TYPE_PLANE, malloc(sizeof(plane_descriptor_t))};
    *((plane_descriptor_t*)node.desc) = std::move(desc);

    aabb_t aabb{};
    return push_primitive_node(std::move(node), std::move(aabb));
}

BPE_API virtual_node_t blobtree_new_virtual_node(const sphere_descriptor_t& desc)
{
    primitive_node_t node{PRIMITIVE_TYPE_SPHERE, malloc(sizeof(sphere_descriptor_t))};
    *((sphere_descriptor_t*)node.desc) = std::move(desc);

    raw_vector3d_t min = desc.center - desc.radius;
    raw_vector3d_t max = desc.center + desc.radius;
    aabb_t         aabb{min, max};
    return push_primitive_node(std::move(node), std::move(aabb));
}

BPE_API virtual_node_t blobtree_new_virtual_node(const cylinder_descriptor_t& desc)
{
    primitive_node_t node{PRIMITIVE_TYPE_CYLINDER, malloc(sizeof(cylinder_descriptor_t))};
    *((cylinder_descriptor_t*)node.desc) = std::move(desc);

    // NOTE: A rough AABB bounding box
    aabb_t aabb{};
    aabb.extend(desc.bottom_origion + desc.radius);
    aabb.extend(desc.bottom_origion - desc.radius);
    aabb.extend(desc.bottom_origion + desc.offset + desc.radius);
    aabb.extend(desc.bottom_origion + desc.offset - desc.radius);
    return push_primitive_node(std::move(node), std::move(aabb));
}

BPE_API virtual_node_t blobtree_new_virtual_node(const cone_descriptor_t& desc)
{
    primitive_node_t node{PRIMITIVE_TYPE_CONE, malloc(sizeof(cone_descriptor_t))};
    *((cone_descriptor_t*)node.desc) = std::move(desc);

    // NOTE: A rough AABB bounding box
    aabb_t aabb{};
    aabb.extend(desc.top_point + desc.radius1);
    aabb.extend(desc.top_point - desc.radius1);
    aabb.extend(desc.bottom_point + desc.radius2);
    aabb.extend(desc.bottom_point - desc.radius2);
    return push_primitive_node(std::move(node), std::move(aabb));
}

BPE_API virtual_node_t blobtree_new_virtual_node(const box_descriptor_t& desc)
{
    primitive_node_t node{PRIMITIVE_TYPE_BOX, malloc(sizeof(box_descriptor_t))};
    *((box_descriptor_t*)node.desc) = std::move(desc);

    raw_vector3d_t min = desc.center - desc.half_size;
    raw_vector3d_t max = desc.center + desc.half_size;
    aabb_t         aabb{min, max};
    return push_primitive_node(std::move(node), std::move(aabb));
}

BPE_API virtual_node_t blobtree_new_virtual_node(const mesh_descriptor_t& desc)
{
    primitive_node_t node{PRIMITIVE_TYPE_MESH, malloc(sizeof(mesh_descriptor_t))};
    *((mesh_descriptor_t*)node.desc) = std::move(desc);

    aabb_t aabb{};
    for (int i = 0; i < desc.point_number; i++) { aabb.extend(desc.points[i]); }

    return push_primitive_node(std::move(node), std::move(aabb));
}

BPE_API virtual_node_t blobtree_new_virtual_node(const extrude_descriptor_t& desc)
{
    primitive_node_t node{PRIMITIVE_TYPE_EXTRUDE, malloc(sizeof(extrude_descriptor_t))};
    *((extrude_descriptor_t*)node.desc) = std::move(desc);

    aabb_t aabb{};
    // NOTE: Currently only straight edges are considered
    for (int i = 0; i < desc.edges_number; i++) {
        aabb.extend(desc.points[i]);
        aabb.extend(desc.points[i] + desc.extusion);
    }

    return push_primitive_node(std::move(node), std::move(aabb));
}

BPE_API void blobtree_free_virtual_node(virtual_node_t* node) { free_sub_blobtree(node->main_index); }

/* Geometry Operations */

BPE_API void virtual_node_boolean_union(virtual_node_t* node1, virtual_node_t* node2)
{
    auto new_node2 = copy(*node2, *node1);

    node_t temp = standard_new_node;
    set_is_primitive(temp, false);
    set_operation(temp, 0);
    set_left_child_index(temp, node1->inner_index);
    set_right_child_index(temp, new_node2.inner_index);

    structures[node1->main_index].nodes.push_back(temp);
    uint32_t parent_index = structures[node1->main_index].nodes.size() - 1;

    set_parent_index(structures[node1->main_index].nodes[node1->inner_index], parent_index);
    set_parent_index(structures[new_node2.main_index].nodes[new_node2.inner_index], parent_index);

    node1->inner_index = parent_index;
}

BPE_API void virtual_node_boolean_intersect(virtual_node_t* node1, virtual_node_t* node2)
{
    auto new_node2 = copy(*node2, *node1);

    node_t temp = standard_new_node;
    set_is_primitive(temp, false);
    set_operation(temp, 1);
    set_left_child_index(temp, node1->inner_index);
    set_right_child_index(temp, new_node2.inner_index);

    structures[node1->main_index].nodes.push_back(temp);
    uint32_t parent_index = structures[node1->main_index].nodes.size() - 1;

    set_parent_index(structures[node1->main_index].nodes[node1->inner_index], parent_index);
    set_parent_index(structures[new_node2.main_index].nodes[new_node2.inner_index], parent_index);

    node1->inner_index = parent_index;
}

BPE_API void virtual_node_boolean_difference(virtual_node_t* node1, virtual_node_t* node2)
{
    auto new_node2 = copy(*node2, *node1);

    node_t temp = standard_new_node;
    set_is_primitive(temp, false);
    set_operation(temp, 2);
    set_left_child_index(temp, node1->inner_index);
    set_right_child_index(temp, new_node2.inner_index);

    structures[node1->main_index].nodes.push_back(temp);
    uint32_t parent_index = structures[node1->main_index].nodes.size() - 1;

    set_parent_index(structures[node1->main_index].nodes[node1->inner_index], parent_index);
    set_parent_index(structures[new_node2.main_index].nodes[new_node2.inner_index], parent_index);

    node1->inner_index = parent_index;
}

BPE_API void virtual_node_offset(virtual_node_t* node, const raw_vector3d_t& direction, const double length)
{
    raw_vector3d_t offset = {direction.x * length, direction.y * length, direction.z * length};
    virtual_node_offset(node, offset);
}

BPE_API void virtual_node_offset(virtual_node_t* node, const raw_vector3d_t& offset)
{
    auto& all_leaf = structures[node->main_index].leaf_index;
    for (int i = 0; i < all_leaf.size(); i++) {
        offset_primitive(primitives[node_fetch_primitive_index(structures[node->main_index].nodes[all_leaf[i]])], offset);

        aabbs[node_fetch_primitive_index(structures[node->main_index].nodes[all_leaf[i]])].offset(offset);
    }
}

BPE_API void virtual_node_split(virtual_node_t* node, raw_vector3d_t base_point, raw_vector3d_t normal)
{
    plane_descriptor_t descriptor;
    descriptor.normal = raw_vector3d_t{normal.x * -1, normal.y * -1, normal.z * -1};
    descriptor.point  = base_point;
    auto plane        = blobtree_new_virtual_node(descriptor);

    virtual_node_boolean_intersect(node, &plane);
}

/* Tree Node Operations */

BPE_API bool virtual_node_set_parent(virtual_node_t* node, virtual_node_t* parent)
{
    // The node's parent is not empty
    if (!is_parent_null(structures[node->main_index].nodes[node->inner_index])) { return false; }

    // The parent's left child is empty
    if (is_left_null(structures[parent->main_index].nodes[parent->inner_index])) {
        // On the same tree
        if (node->main_index == parent->main_index) {
            set_parent_index(structures[node->main_index].nodes[node->inner_index], parent->inner_index);
            set_left_child_index(structures[parent->main_index].nodes[parent->inner_index], node->inner_index);
        } else {
            auto new_parent = copy(*parent, *node);
            set_parent_index(structures[node->main_index].nodes[node->inner_index], new_parent.inner_index);
            set_left_child_index(structures[new_parent.main_index].nodes[new_parent.inner_index], node->inner_index);
        }
    }
    // The parent's right child is empty
    else if (is_right_null(structures[parent->main_index].nodes[parent->inner_index])) {
        // On the same tree
        if (node->main_index == parent->main_index) {
            set_parent_index(structures[node->main_index].nodes[node->inner_index], parent->inner_index);
            set_right_child_index(structures[parent->main_index].nodes[parent->inner_index], node->inner_index);
        } else {
            auto new_parent = copy(*parent, *node);
            set_parent_index(structures[node->main_index].nodes[node->inner_index], new_parent.inner_index);
            set_right_child_index(structures[new_parent.main_index].nodes[new_parent.inner_index], node->inner_index);
        }
    } else {
        return false;
    }
}

BPE_API bool virtual_node_set_left_child(virtual_node_t* node, virtual_node_t* child)
{
    // The child's parent is not empty
    if (!is_parent_null(structures[child->main_index].nodes[child->inner_index])) { return false; }

    // The node's left child is not empty
    if (!is_left_null(structures[node->main_index].nodes[node->inner_index])) { return false; }

    // On the same tree
    if (node->main_index == child->main_index) {
        set_parent_index(structures[child->main_index].nodes[child->inner_index], node->inner_index);
        set_left_child_index(structures[node->main_index].nodes[node->inner_index], child->inner_index);
    } else {
        auto new_child = copy(*child, *node);
        set_parent_index(structures[new_child.main_index].nodes[new_child.inner_index], node->inner_index);
        set_left_child_index(structures[node->main_index].nodes[node->inner_index], new_child.inner_index);
    }
}

BPE_API bool virtual_node_set_right_child(virtual_node_t* node, virtual_node_t* child)
{
    // The child's parent is not empty
    if (!is_parent_null(structures[child->main_index].nodes[child->inner_index])) { return false; }

    // The node's right child is not empty
    if (!is_right_null(structures[node->main_index].nodes[node->inner_index])) { return false; }

    // On the same tree
    if (node->main_index == child->main_index) {
        set_parent_index(structures[child->main_index].nodes[child->inner_index], node->inner_index);
        set_right_child_index(structures[node->main_index].nodes[node->inner_index], child->inner_index);
    } else {
        auto new_child = copy(*child, *node);
        set_parent_index(structures[new_child.main_index].nodes[new_child.inner_index], node->inner_index);
        set_right_child_index(structures[node->main_index].nodes[node->inner_index], new_child.inner_index);
    }
}

BPE_API bool virtual_node_add_child(virtual_node_t* node, virtual_node_t* child)
{
    if (virtual_node_set_left_child(node, child)) {
        return true;
    } else if (virtual_node_set_right_child(node, child)) {
        return true;
    } else {
        return false;
    }
}

BPE_API bool virtual_node_remove_child(virtual_node_t* node, virtual_node_t* child)
{
    if (node->main_index != child->main_index) { return false; }

    if (node_fetch_left_child_index(structures[node->main_index].nodes[node->inner_index]) == child->inner_index) {
        set_left_child_index(structures[node->main_index].nodes[node->inner_index], 0xFFFFFFFFu);
        blobtree_free_virtual_node(child);
        return true;
    } else if (node_fetch_right_child_index(structures[node->main_index].nodes[node->inner_index]) == child->inner_index) {
        set_right_child_index(structures[node->main_index].nodes[node->inner_index], 0xFFFFFFFFu);
        blobtree_free_virtual_node(child);
        return true;
    } else {
        return false;
    }
}

/* Node Replacement Operation */

BPE_API bool virtual_node_replace_primitive(virtual_node_t* node, const constant_descriptor_t& desc)
{
    if (!is_primitive_node(structures[node->main_index].nodes[node->inner_index])) { return false; }
    *((constant_descriptor_t*)primitives[node_fetch_primitive_index(structures[node->main_index].nodes[node->inner_index])]
          .desc) = std::move(desc);
    primitives[node_fetch_primitive_index(structures[node->main_index].nodes[node->inner_index])].type =
        PRIMITIVE_TYPE_CONSTANT;
    aabbs[node_fetch_primitive_index(structures[node->main_index].nodes[node->inner_index])] = aabb_t{};
    return true;
}

BPE_API bool virtual_node_replace_primitive(virtual_node_t* node, const plane_descriptor_t& desc)
{
    if (!is_primitive_node(structures[node->main_index].nodes[node->inner_index])) { return false; }
    *((plane_descriptor_t*)primitives[node_fetch_primitive_index(structures[node->main_index].nodes[node->inner_index])].desc) =
        std::move(desc);
    primitives[node_fetch_primitive_index(structures[node->main_index].nodes[node->inner_index])].type = PRIMITIVE_TYPE_PLANE;
    aabbs[node_fetch_primitive_index(structures[node->main_index].nodes[node->inner_index])]           = aabb_t{};
    return true;
}

BPE_API bool virtual_node_replace_primitive(virtual_node_t* node, const sphere_descriptor_t& desc)
{
    if (!is_primitive_node(structures[node->main_index].nodes[node->inner_index])) { return false; }
    *((sphere_descriptor_t*)primitives[node_fetch_primitive_index(structures[node->main_index].nodes[node->inner_index])]
          .desc)                                                                                       = std::move(desc);
    primitives[node_fetch_primitive_index(structures[node->main_index].nodes[node->inner_index])].type = PRIMITIVE_TYPE_SPHERE;

    raw_vector3d_t min = desc.center - desc.radius;
    raw_vector3d_t max = desc.center + desc.radius;
    aabb_t         aabb{min, max};
    aabbs[node_fetch_primitive_index(structures[node->main_index].nodes[node->inner_index])] = aabb;

    return true;
}

BPE_API bool virtual_node_replace_primitive(virtual_node_t* node, const cylinder_descriptor_t& desc)
{
    if (!is_primitive_node(structures[node->main_index].nodes[node->inner_index])) { return false; }
    *((cylinder_descriptor_t*)primitives[node_fetch_primitive_index(structures[node->main_index].nodes[node->inner_index])]
          .desc) = std::move(desc);
    primitives[node_fetch_primitive_index(structures[node->main_index].nodes[node->inner_index])].type =
        PRIMITIVE_TYPE_CYLINDER;

    // NOTE: A rough AABB bounding box
    aabb_t aabb{};
    aabb.extend(desc.bottom_origion + desc.radius);
    aabb.extend(desc.bottom_origion - desc.radius);
    aabb.extend(desc.bottom_origion + desc.offset + desc.radius);
    aabb.extend(desc.bottom_origion + desc.offset - desc.radius);
    aabbs[node_fetch_primitive_index(structures[node->main_index].nodes[node->inner_index])] = aabb;

    return true;
}

BPE_API bool virtual_node_replace_primitive(virtual_node_t* node, const cone_descriptor_t& desc)
{
    if (!is_primitive_node(structures[node->main_index].nodes[node->inner_index])) { return false; }
    *((cone_descriptor_t*)primitives[node_fetch_primitive_index(structures[node->main_index].nodes[node->inner_index])].desc) =
        std::move(desc);
    primitives[node_fetch_primitive_index(structures[node->main_index].nodes[node->inner_index])].type = PRIMITIVE_TYPE_CONE;

    // NOTE: A rough AABB bounding box
    aabb_t aabb{};
    aabb.extend(desc.top_point + desc.radius1);
    aabb.extend(desc.top_point - desc.radius1);
    aabb.extend(desc.bottom_point + desc.radius2);
    aabb.extend(desc.bottom_point - desc.radius2);
    aabbs[node_fetch_primitive_index(structures[node->main_index].nodes[node->inner_index])] = aabb;
    return true;
}

BPE_API bool virtual_node_replace_primitive(virtual_node_t* node, const box_descriptor_t& desc)
{
    if (!is_primitive_node(structures[node->main_index].nodes[node->inner_index])) { return false; }
    *((box_descriptor_t*)primitives[node_fetch_primitive_index(structures[node->main_index].nodes[node->inner_index])].desc) =
        std::move(desc);
    primitives[node_fetch_primitive_index(structures[node->main_index].nodes[node->inner_index])].type = PRIMITIVE_TYPE_BOX;

    raw_vector3d_t min = desc.center - desc.half_size;
    raw_vector3d_t max = desc.center + desc.half_size;
    aabb_t         aabb{min, max};
    aabbs[node_fetch_primitive_index(structures[node->main_index].nodes[node->inner_index])] = aabb;
    return true;
}

BPE_API bool virtual_node_replace_primitive(virtual_node_t* node, const mesh_descriptor_t& desc)
{
    if (!is_primitive_node(structures[node->main_index].nodes[node->inner_index])) { return false; }
    *((mesh_descriptor_t*)primitives[node_fetch_primitive_index(structures[node->main_index].nodes[node->inner_index])].desc) =
        std::move(desc);
    primitives[node_fetch_primitive_index(structures[node->main_index].nodes[node->inner_index])].type = PRIMITIVE_TYPE_MESH;

    aabb_t aabb{};
    for (int i = 0; i < desc.point_number; i++) { aabb.extend(desc.points[i]); }

    aabbs[node_fetch_primitive_index(structures[node->main_index].nodes[node->inner_index])] = aabb;
    return true;
}

BPE_API bool virtual_node_replace_primitive(virtual_node_t* node, const extrude_descriptor_t& desc)
{
    if (!is_primitive_node(structures[node->main_index].nodes[node->inner_index])) { return false; }
    *((extrude_descriptor_t*)primitives[node_fetch_primitive_index(structures[node->main_index].nodes[node->inner_index])]
          .desc)                                                                                       = std::move(desc);
    primitives[node_fetch_primitive_index(structures[node->main_index].nodes[node->inner_index])].type = PRIMITIVE_TYPE_EXTRUDE;

    aabb_t aabb{};
    // NOTE: Currently only straight edges are considered
    for (int i = 0; i < desc.edges_number; i++) {
        aabb.extend(desc.points[i]);
        aabb.extend(desc.points[i] + desc.extusion);
    }
    aabbs[node_fetch_primitive_index(structures[node->main_index].nodes[node->inner_index])] = aabb;
    return true;
}

#ifdef _DEBUG

void output_primitive_node(const primitive_node_t& node)
{
    auto output_point = [](const raw_vector3d_t& point) {
        std::cout << "( " << point.x << ", " << point.y << ", " << point.z << " )" << std::endl;
    };

    auto type = node.type;
    switch (type) {
        case PRIMITIVE_TYPE_CONSTANT: {
            auto desc = static_cast<constant_descriptor_t*>(node.desc);
            std::cout << "constant:" << std::endl;
            std::cout << "\tvalue: " << desc->value << std::endl << std::endl;
            break;
        }
        case PRIMITIVE_TYPE_PLANE: {
            auto desc = static_cast<plane_descriptor_t*>(node.desc);
            std::cout << "plane:" << std::endl;
            std::cout << "\tbase point: ";
            output_point(desc->point);
            std::cout << "\tnormal: ";
            output_point(desc->normal);
            std::cout << std::endl;
            break;
        }
        case PRIMITIVE_TYPE_SPHERE: {
            auto desc = static_cast<sphere_descriptor_t*>(node.desc);
            std::cout << "sphere:" << std::endl;
            std::cout << "\tcenter: ";
            output_point(desc->center);
            std::cout << "\tradius: " << desc->radius << std::endl << std::endl;
            break;
        }
        case PRIMITIVE_TYPE_CYLINDER: {
            auto desc = static_cast<cylinder_descriptor_t*>(node.desc);
            std::cout << "cylinder:" << std::endl;
            std::cout << "\tbottom point: ";
            output_point(desc->bottom_origion);
            std::cout << "\tradius: " << desc->radius << std::endl << std::endl;
            std::cout << "\toffset: ";
            output_point(desc->offset);
            break;
        }
        case PRIMITIVE_TYPE_CONE: {
            auto desc = static_cast<cone_descriptor_t*>(node.desc);
            std::cout << "cone:" << std::endl;
            std::cout << "\tbottom point: ";
            output_point(desc->bottom_point);
            std::cout << "\ttop point: ";
            output_point(desc->top_point);
            std::cout << "\tradius1: " << desc->radius1 << std::endl;
            std::cout << "\tradius2: " << desc->radius2 << std::endl << std::endl;
            break;
        }
        case PRIMITIVE_TYPE_BOX: {
            auto desc = static_cast<box_descriptor_t*>(node.desc);
            std::cout << "box:" << std::endl;
            std::cout << "\tcenter: ";
            output_point(desc->center);
            std::cout << "\thalf_size ";
            output_point(desc->half_size);
            break;
        }
        case PRIMITIVE_TYPE_MESH: {
            auto desc = static_cast<mesh_descriptor_t*>(node.desc);
            std::cout << "mesh:" << std::endl;
            std::cout << "\tpoint number: " << desc->point_number << std::endl;
            for (int i = 0; i < desc->point_number; i++) {
                std::cout << "\t\t( " << desc->points[i].x << ", " << desc->points[i].y << ", " << desc->points[i].z << " )"
                          << std::endl;
            }

            std::cout << "\tfaces number: " << desc->face_number << std::endl;
            for (int i = 0; i < desc->face_number; i++) {
                auto begin  = desc->faces[i][0];
                auto length = desc->faces[i][1];
                std::cout << "\t\t<" << begin << ", " << length << "> : ";
                for (int j = begin; j < begin + length; j++) { std::cout << desc->indexs[j] << " "; }
                std::cout << std::endl;
            }
            break;
        }
        case PRIMITIVE_TYPE_EXTRUDE: {
            auto desc = static_cast<extrude_descriptor_t*>(node.desc);
            std::cout << "extrude:" << std::endl;
            std::cout << "\tedges number: " << desc->edges_number << std::endl;
            std::cout << "\textusion: ";
            output_point(desc->extusion);

            std::cout << "\tpoints: " << std::endl;
            for (int i = 0; i < desc->edges_number; i++) {
                std::cout << "\t\t( " << desc->points[i].x << ", " << desc->points[i].y << ", " << desc->points[i].z << " )"
                          << std::endl;
            }

            std::cout << "\tbulges: " << std::endl;
            for (int i = 0; i < desc->edges_number; i++) { std::cout << "\t\t" << desc->bulges[i] << std::endl; }
            break;
        }
        default: {
            break;
        }
    }
}

void output_blobtree(virtual_node_t node)
{
    std::map<int, std::string> index;
    index[0] = "constant";
    index[1] = "plane";
    index[2] = "sphere";
    index[3] = "cylinder";
    index[4] = "cone";
    index[5] = "box";
    index[6] = "mesh";
    index[7] = "extrude";

    auto               root = structures[node.main_index].nodes[node.inner_index];
    std::queue<node_t> now, next;
    now.push(root);

    std::vector<primitive_node_t> temp;

    while (!now.empty()) {
        auto begin = now.front();
        now.pop();

        if (is_primitive_node(begin)) {
            std::cout << index[primitives[node_fetch_primitive_index(begin)].type] << "\t\t";
            temp.push_back(primitives[node_fetch_primitive_index(begin)]);
        } else {
            auto op = (uint32_t)node_fetch_operation(begin);
            if (op == 0) {
                std::cout << "or"
                          << "\t\t";
            } else if (op == 1) {
                std::cout << "and"
                          << "\t\t";
            } else if (op == 2) {
                std::cout << "sub"
                          << "\t\t";
            }
        }

        if (!is_left_null(begin)) { next.push(structures[node.main_index].nodes[node_fetch_left_child_index(begin)]); }
        if (!is_right_null(begin)) { next.push(structures[node.main_index].nodes[node_fetch_right_child_index(begin)]); }

        if (now.empty()) {
            now = next;
            while (!next.empty()) { next.pop(); }
            std::cout << std::endl;
        }
    }
    std::cout << std::endl;

    for (int i = 0; i < temp.size(); i++) { output_primitive_node(temp[i]); }
}
#endif // _DEBUG
