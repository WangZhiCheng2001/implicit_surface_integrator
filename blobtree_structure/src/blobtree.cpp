#include <cassert>

#include "internal_api.hpp"

#include "globals.hpp"
#include "primitive_node_destroyer.hpp"

/* internal global variables for blobtree */
std::vector<blobtree_t, tbb::tbb_allocator<blobtree_t>> structures{};
std::vector<bool, tbb::tbb_allocator<bool>>             use_mark{};

std::vector<primitive_node_t, tbb::tbb_allocator<primitive_node_t>> primitives{};
std::vector<aabb_t, tbb::tbb_allocator<aabb_t>>                     aabbs{};
std::vector<uint32_t, tbb::tbb_allocator<uint32_t>>                 counter{};

std::stack<uint32_t, std::deque<uint32_t, tbb::tbb_allocator<uint32_t>>> free_structure_list{};

/* =============================================================================================
 * basic functionalities
 * ============================================================================================= */

BS_API size_t blobtree_get_node_count(uint32_t index) noexcept { return structures[index].nodes.size(); }

BS_API std::vector<uint32_t, tbb::tbb_allocator<uint32_t>> blobtree_get_leaf_nodes(uint32_t index) noexcept
{
    return structures[index].leaf_index;
}

BS_API node_t& blobtree_get_node(const virtual_node_t& node) noexcept
{
    return structures[node.main_index].nodes[node.inner_index];
}

BS_API size_t get_primitive_count(const virtual_node_t& tree_node) noexcept
{
    assert(primitives.size() == aabbs.size());
    return structures[tree_node.main_index].leaf_index.size();
}

BS_API std::vector<uint32_t, tbb::tbb_allocator<uint32_t>> get_all_primitive_index(const virtual_node_t& tree_node)
{
    std::vector<uint32_t, tbb::tbb_allocator<uint32_t>> result;
    auto&                                               tree = structures[tree_node.main_index];
    for (auto& index : tree.leaf_index) { result.push_back(node_fetch_primitive_index(tree.nodes[index])); }
    return std::move(result);
}

BS_API std::vector<Eigen::Vector3d, tbb::tbb_allocator<Eigen::Vector3d>> get_all_primitive_offset(
    const virtual_node_t& tree_node)
{
    std::vector<Eigen::Vector3d, tbb::tbb_allocator<Eigen::Vector3d>> result;
    auto&                                                             tree = structures[tree_node.main_index];
    for (auto& index : tree.leaf_index) {
        auto& node = tree.nodes[index];
        if (node_is_transform_null(node)) {
            result.push_back(Eigen::Vector3d{0.0, 0.0, 0.0});
        } else {
            result.push_back(tree.transform[node_fetch_transform_index(node)].block<3, 1>(0, 3));
        }
    }
    return std::move(result);
}

BS_API const primitive_node_t& get_primitive_node(uint32_t index) noexcept
{
    assert(index < primitives.size());
    return primitives[index];
}

BS_API const aabb_t& get_aabb(uint32_t index) noexcept
{
    assert(index < aabbs.size());
    return aabbs[index];
}

void shrink_primitives() { primitives.shrink_to_fit(); }

uint32_t get_next_available_blobtree_index()
{
    auto index = std::find(use_mark.begin(), use_mark.end(), false);
    if (index == use_mark.end()) {
        structures.resize(structures.size() + 1);
        use_mark.push_back(false);
        return structures.size() - 1;
    } else {
        return *index;
    }
}

void blobtree_merge_update_index(blobtree_t& old_tree, virtual_node_t new_node)
{
    const auto size = static_cast<uint32_t>(structures[new_node.main_index].nodes.size());
    for (uint32_t i = 0; i < old_tree.nodes.size(); i++) {
        if (!node_is_parent_null(old_tree.nodes[i])) node_fetch_parent_index(old_tree.nodes[i]) += size;
        if (!node_is_left_child_null(old_tree.nodes[i])) node_fetch_left_child_index(old_tree.nodes[i]) += size;
        if (!node_is_right_child_null(old_tree.nodes[i])) node_fetch_right_child_index(old_tree.nodes[i]) += size;
    }
    for (uint32_t i = 0; i < old_tree.leaf_index.size(); i++) { old_tree.leaf_index[i] += size; }

    const auto transform_size = static_cast<uint32_t>(structures[new_node.main_index].transform.size());
    for (uint32_t i = 0; i < old_tree.leaf_index.size(); i++) {
        if (!node_is_transform_null(old_tree.nodes[old_tree.leaf_index[i]])) {
            node_fetch_transform_index(old_tree.nodes[old_tree.leaf_index[i]]) += transform_size;
        }
    }
}

virtual_node_t copy(virtual_node_t old_node, virtual_node_t new_node)
{
    assert(old_node.main_index != new_node.main_index);
    // Incrementing the reference count
    for (auto& index : structures[old_node.main_index].leaf_index) {
        counter[node_fetch_primitive_index(structures[old_node.main_index].nodes[index])]++;
    }

    // Copy a tree and its subtrees to a temporary tree
    const auto size = static_cast<uint32_t>(structures[new_node.main_index].nodes.size());
    auto       temp = structures[old_node.main_index];

    // Update all index
    blobtree_merge_update_index(temp, new_node);

    // Copy the updated index tree to the array at the new location
    structures[new_node.main_index].nodes.insert(structures[new_node.main_index].nodes.end(),
                                                 temp.nodes.begin(),
                                                 temp.nodes.end());
    structures[new_node.main_index].leaf_index.insert(structures[new_node.main_index].leaf_index.end(),
                                                      temp.leaf_index.begin(),
                                                      temp.leaf_index.end());
    structures[new_node.main_index].transform.insert(structures[new_node.main_index].transform.end(),
                                                     temp.transform.begin(),
                                                     temp.transform.end());

    return virtual_node_t{new_node.main_index, old_node.inner_index + size};
}

virtual_node_t move(virtual_node_t old_node, virtual_node_t new_node)
{
    assert(old_node.main_index != new_node.main_index);

    // Get a reference of a tree and its subtrees
    const auto size = static_cast<uint32_t>(structures[new_node.main_index].nodes.size());
    auto&      old  = structures[old_node.main_index];

    // Update all index
    blobtree_merge_update_index(old, new_node);

    // Mode the updated index tree to the array at the new location
    structures[new_node.main_index].nodes.insert(structures[new_node.main_index].nodes.end(),
                                                 std::make_move_iterator(old.nodes.begin()),
                                                 std::make_move_iterator(old.nodes.end()));
    structures[new_node.main_index].leaf_index.insert(structures[new_node.main_index].leaf_index.end(),
                                                      std::make_move_iterator(old.leaf_index.begin()),
                                                      std::make_move_iterator(old.leaf_index.end()));
    structures[new_node.main_index].transform.insert(structures[new_node.main_index].transform.end(),
                                                     std::make_move_iterator(old.transform.begin()),
                                                     std::make_move_iterator(old.transform.end()));

    return virtual_node_t{new_node.main_index, old_node.inner_index + size};
}

BS_API void free_sub_blobtree(uint32_t index) noexcept
{
    // 这里尽量打标记，延迟修改和删除
    free_structure_list.push(index);
}

BS_API void free_blobtree(virtual_node_t node)
{
    use_mark[node.main_index] = false;
    auto& tree                = structures[node.main_index];

    for (auto& index : tree.leaf_index) { counter[node_fetch_primitive_index(tree.nodes[index])]--; }

    tree.leaf_index.clear();
    tree.nodes.clear();
    tree.transform.clear();
}

BS_API void clear_blobtree() noexcept
{
    structures.clear();
    aabbs.clear();
    for (auto& prim : primitives) { destroy_primitive_node(prim); }
    primitives.clear();
    while (!free_structure_list.empty()) free_structure_list.pop();
}

// bool upward_propagation(blobtree_t& tree, const int leaf_node_index, const int root_index)
// {
//     int now_index = leaf_node_index;
//     now_index     = node_fetch_parent_index(tree.nodes[now_index]);

//     while (true) {
//         auto& node        = tree.nodes[now_index];
//         auto& left_child  = tree.nodes[node_fetch_left_child_index(node)];
//         auto& right_child = tree.nodes[node_fetch_right_child_index(node)];

//         auto          node_in_out_flag        = node_fetch_in_out(node);
//         eNodeLocation left_child_in_out_flag  = node_fetch_in_out(left_child);
//         eNodeLocation right_child_in_out_flag = node_fetch_in_out(right_child);
//         if (node_in_out_flag != eNodeLocation::unset) { return false; }

//         switch (node_fetch_operation(node)) {
//             case eNodeOperation::unionOp: {
//                 if (left_child_in_out_flag == eNodeLocation::in || right_child_in_out_flag == eNodeLocation::in) {
//                     node_in_out_flag = eNodeLocation::in;
//                 } else if (left_child_in_out_flag == eNodeLocation::out && right_child_in_out_flag ==
//                 eNodeLocation::out) {
//                     node_in_out_flag = eNodeLocation::out;
//                 } else {
//                     return false;
//                 }
//                 break;
//             }
//             case eNodeOperation::intersectionOp: {
//                 if (left_child_in_out_flag == eNodeLocation::in && right_child_in_out_flag == eNodeLocation::in) {
//                     node_in_out_flag = eNodeLocation::in;
//                 } else if (left_child_in_out_flag == eNodeLocation::out || right_child_in_out_flag ==
//                 eNodeLocation::out) {
//                     node_in_out_flag = eNodeLocation::out;
//                 } else {
//                     return false;
//                 }
//                 break;
//             }
//             case eNodeOperation::differenceOp: {
//                 if (left_child_in_out_flag == eNodeLocation::in && right_child_in_out_flag == eNodeLocation::out) {
//                     node_in_out_flag = eNodeLocation::in;
//                 }
//                 if (left_child_in_out_flag == eNodeLocation::out || right_child_in_out_flag == eNodeLocation::in) {
//                     node_in_out_flag = eNodeLocation::out;
//                 } else {
//                     return false;
//                 }
//                 break;
//             }
//             default: {
//                 return false;
//                 break;
//             }
//         }

//         if (now_index == root_index) { return true; }

//         now_index = node_fetch_parent_index(node);
//     }
// }

// eNodeLocation evaluate(const virtual_node_t& node, const raw_vector3d_t& point)
//{
//     auto  tree       = structures[node.main_index];
//     auto& leaf_index = tree.leaf_index;
//     for (auto& index : leaf_index) {
//         auto sdf              = evaluate(primitives[node_fetch_primitive_index(tree.nodes[index])], point);
//         auto leaf_node_in_out = node_fetch_in_out(tree.nodes[index]);
//         if (sdf <= 0.0)
//             leaf_node_in_out = eNodeLocation::in;
//         else
//             leaf_node_in_out = eNodeLocation::out;
//         if (upward_propagation(tree, index, node.inner_index)) { break; }
//     }
//     return node_fetch_in_out(tree.nodes[node.inner_index]);
// }

// aabb_t get_aabb(const virtual_node_t& node)
// {
//     auto&  tree       = structures[node.main_index];
//     auto&  leaf_index = tree.leaf_index;
//     aabb_t result{};
//     for (auto& index : leaf_index) {
//         auto& type = primitives[node_fetch_primitive_index(tree.nodes[index])].type;
//         if (type != PRIMITIVE_TYPE_CONSTANT && type != PRIMITIVE_TYPE_PLANE) { result.extend(aabbs[index]); }
//     }
//     return result;
// }

// uint32_t get_closest_common_parent(const std::vector<bool>& mask, const int main_index)
// {
//     // Copy tree structure
//     auto tree = structures[main_index];

//     // Count how many geometries are queried
//     int count = 0;
//     for (int i = 0; i < mask.size(); i++) {
//         if (mask[i] == 1) { count++; }
//     }

//     uint32_t result = 0xFFFFFFFFu;
//     for (uint32_t i = 0; i < mask.size(); i++) {
//         if (mask[i] == 0) { continue; }

//         for (auto& iter : tree.leaf_index) {
//             // Find the location of the current query geometry in the tree
//             if (node_fetch_primitive_index(tree.nodes[iter]) != i) { continue; }

//             // Traverse from bottom to top and increase the count of all nodes by 1
//             uint32_t now = iter;
//             while (true) {
//                 now = node_fetch_parent_index(tree.nodes[now]);

//                 // now is root
//                 if (now == 0xFFFFFFFFu) { break; }

//                 // Use the primitive index of the internal node to count
//                 if (node_fetch_primitive_index(tree.nodes[now]) == 0xFFFFFFu) {
//                     set_primitive_index(tree.nodes[now], 1);
//                 } else {
//                     set_primitive_index(tree.nodes[now], node_fetch_primitive_index(tree.nodes[now]) + 1);
//                 }

//                 if (node_fetch_primitive_index(tree.nodes[now]) == count) {
//                     result = now;
//                     break;
//                 }
//             }

//             break;
//         }
//     }

//     return result;
// }

/* =============================================================================================
 * tree node operations
 * ============================================================================================= */

BS_API bool virtual_node_set_parent(const virtual_node_t& node, const virtual_node_t& parent)
{
    auto& node_in_tree = structures[node.main_index].nodes[node.inner_index];
    // The node's parent is not empty
    if (!node_is_parent_null(node_in_tree)) { return false; }

    auto& parent_in_tree     = structures[parent.main_index].nodes[parent.inner_index];
    auto  parent_inner_index = parent.inner_index;
    // not on the same tree
    if (node.main_index != parent.main_index) {
        auto new_parent    = copy(parent, node);
        parent_in_tree     = structures[new_parent.main_index].nodes[new_parent.inner_index];
        parent_inner_index = new_parent.inner_index;
    }
    auto parent_left_child  = node_fetch_left_child_index(parent_in_tree);
    auto parent_right_child = node_fetch_right_child_index(parent_in_tree);

    // set parent index
    node_fetch_parent_index(node_in_tree) = parent_inner_index;

    // The parent's left child is empty
    if (node_is_left_child_null(node_in_tree)) {
        parent_left_child = node.inner_index;
        return true;
    }
    // The parent's right child is empty
    else if (node_is_right_child_null(node_in_tree)) {
        parent_right_child = node.inner_index;
        return true;
    }

    return false;
}

BS_API bool virtual_node_set_left_child(const virtual_node_t& node, const virtual_node_t& child)
{
    auto& node_in_tree = structures[node.main_index].nodes[node.inner_index];

    // The child's parent is not empty
    if (!node_is_parent_null(node_in_tree)) { return false; }

    // The node's left child is not empty
    if (!node_is_left_child_null(node_in_tree)) { return false; }

    auto node_left_child  = node_fetch_left_child_index(node_in_tree);
    auto node_right_child = node_fetch_right_child_index(node_in_tree);

    // On the same tree
    if (node.main_index == child.main_index) {
        auto& child_in_tree                    = structures[child.main_index].nodes[child.inner_index];
        node_fetch_parent_index(child_in_tree) = node.inner_index;
        node_left_child                        = child.inner_index;
    } else {
        auto  new_child                        = copy(child, node);
        auto& child_in_tree                    = structures[new_child.main_index].nodes[new_child.inner_index];
        node_fetch_parent_index(child_in_tree) = node.inner_index;
        node_left_child                        = new_child.inner_index;
    }

    return true;
}

BS_API bool virtual_node_set_right_child(const virtual_node_t& node, const virtual_node_t& child)
{
    auto& node_in_tree = structures[node.main_index].nodes[node.inner_index];

    // The child's parent is not empty
    if (!node_is_parent_null(node_in_tree)) { return false; }

    // The node's right child is not empty
    if (!node_is_right_child_null(node_in_tree)) { return false; }

    auto node_left_child  = node_fetch_left_child_index(node_in_tree);
    auto node_right_child = node_fetch_right_child_index(node_in_tree);

    // On the same tree
    if (node.main_index == child.main_index) {
        auto& child_in_tree                    = structures[child.main_index].nodes[child.inner_index];
        node_fetch_parent_index(child_in_tree) = node.inner_index;
        node_right_child                       = child.inner_index;
    } else {
        auto  new_child                        = copy(child, node);
        auto& child_in_tree                    = structures[new_child.main_index].nodes[new_child.inner_index];
        node_fetch_parent_index(child_in_tree) = node.inner_index;
        node_right_child                       = new_child.inner_index;
    }

    return true;
}

BS_API bool virtual_node_add_child(const virtual_node_t& node, const virtual_node_t& child)
{
    if (virtual_node_set_left_child(node, child)) {
        return true;
    } else if (virtual_node_set_right_child(node, child)) {
        return true;
    }

    return false;
}

BS_API bool virtual_node_remove_child(const virtual_node_t& node, const virtual_node_t& child)
{
    if (node.main_index != child.main_index) { return false; }

    auto& node_in_tree     = structures[node.main_index].nodes[node.inner_index];
    auto  node_left_child  = node_fetch_left_child_index(node_in_tree);
    auto  node_right_child = node_fetch_right_child_index(node_in_tree);

    if (node_left_child == child.inner_index) {
        node_left_child = 0xFFFFFFFFu;
        blobtree_free_virtual_node(child);
        return true;
    } else if (node_right_child == child.inner_index) {
        node_right_child = 0xFFFFFFFFu;
        blobtree_free_virtual_node(child);
        return true;
    }

    return false;
}

/* =============================================================================================
 * geometry operations
 * ============================================================================================= */

static inline virtual_node_t virtual_node_boolean_op(virtual_node_t&       node1,
                                                     const virtual_node_t& node2,
                                                     eNodeOperation        op,
                                                     const bool            keep1 = false,
                                                     const bool            keep2 = false)
{
    virtual_node_t new_node1, new_node2, result_node;

    if (keep1 && keep2) {
        new_node1.main_index           = get_next_available_blobtree_index();
        new_node1.inner_index          = 0;
        new_node1                      = copy(node1, new_node1);
        new_node2                      = copy(new_node2, new_node1);
        use_mark[new_node1.main_index] = true;
    } else if (keep1) {
        new_node2 = node2;
        new_node1 = copy(node1, node2);
    } else if (keep2) {
        new_node1 = node1;
        new_node2 = copy(node1, node2);
    } else {
        new_node1 = node1;
        new_node2 = move(node1, node2);
    }

    auto& inserted_node                         = structures[new_node1.main_index].nodes.emplace_back(standard_new_node);
    node_fetch_is_primitive(inserted_node)      = false;
    // weird bug: need to force cast, or it will be treated as uint32_t instead of eNodeOperation
    node_fetch_operation(inserted_node)         = (eNodeOperation)op;
    node_fetch_left_child_index(inserted_node)  = new_node1.inner_index;
    node_fetch_right_child_index(inserted_node) = new_node2.inner_index;

    uint32_t parent_index = structures[new_node1.main_index].nodes.size() - 1;
    node_fetch_parent_index(structures[new_node1.main_index].nodes[new_node1.inner_index]) = parent_index;
    node_fetch_parent_index(structures[new_node2.main_index].nodes[new_node2.inner_index]) = parent_index;

    result_node.main_index  = new_node1.main_index;
    result_node.inner_index = parent_index;

    return result_node;
}

BS_API virtual_node_t virtual_node_boolean_union(virtual_node_t&       node1,
                                                 const virtual_node_t& node2,
                                                 const bool            keep1 = false,
                                                 const bool            keep2 = false)
{
    return virtual_node_boolean_op(node1, node2, eNodeOperation::unionOp, keep1, keep2);
}

BS_API virtual_node_t virtual_node_boolean_intersect(virtual_node_t&       node1,
                                                     const virtual_node_t& node2,
                                                     const bool            keep1 = false,
                                                     const bool            keep2 = false)
{
    return virtual_node_boolean_op(node1, node2, eNodeOperation::intersectionOp, keep1, keep2);
}

BS_API virtual_node_t virtual_node_boolean_difference(virtual_node_t&       node1,
                                                      const virtual_node_t& node2,
                                                      const bool            keep1 = false,
                                                      const bool            keep2 = false)
{
    return virtual_node_boolean_op(node1, node2, eNodeOperation::differenceOp, keep1, keep2);
}

void offset_primitive_node(virtual_node_t& tree, node_t& primitive_node, const Eigen::Vector3d& offset)
{
    if (node_is_transform_null(primitive_node)) {
        Eigen::Matrix4d transform   = Eigen::Matrix4d::Identity();
        transform.block<3, 1>(0, 3) = offset;
        structures[tree.main_index].transform.push_back(transform);
        node_fetch_transform_index(primitive_node) = structures[tree.main_index].transform.size() - 1;
    } else {
        structures[tree.main_index].transform[node_fetch_transform_index(primitive_node)].block<3, 1>(0, 3) += offset;
    }
}

BS_API void virtual_node_offset(virtual_node_t& node, const raw_vector3d_t& direction, const double length)
{
    raw_vector3d_t offset = {direction.x * length, direction.y * length, direction.z * length};
    virtual_node_offset(node, offset);
}

BS_API void virtual_node_offset(virtual_node_t& node, const raw_vector3d_t& offset)
{
    Eigen::Map<const Eigen::Vector3d> offset_(&offset.x);

    for (const auto& leaf_index : structures[node.main_index].leaf_index) {
        auto& primitive_node = structures[node.main_index].nodes[leaf_index];
        offset_primitive_node(node, primitive_node, offset_);
    }
}

BS_API void virtual_node_split(virtual_node_t& node, raw_vector3d_t base_point, raw_vector3d_t normal)
{
    plane_descriptor_t descriptor;
    descriptor.normal = raw_vector3d_t{normal.x * -1, normal.y * -1, normal.z * -1};
    descriptor.point  = base_point;
    auto plane        = blobtree_new_virtual_node(descriptor);

    virtual_node_boolean_intersect(node, plane);
}