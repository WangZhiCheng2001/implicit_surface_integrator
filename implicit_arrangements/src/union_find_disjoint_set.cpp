#include <numeric>

#include <range/v3/range.hpp>
#include <range/v3/view.hpp>

#include "union_find_disjoint_set.hpp"
#include "robust_assert.hpp"

UnionFindDisjointSet::UnionFindDisjointSet(uint32_t size) noexcept { init(size); }

void UnionFindDisjointSet::init(uint32_t size) noexcept
{
    parent.resize(size);
    std::iota(parent.begin(), parent.end(), 0u);
}

int32_t UnionFindDisjointSet::find(uint32_t x) noexcept
{
    while (parent[x] != x) {
        parent[x] = parent[parent[x]];
        x         = parent[x];
    }
    return x;
}

int32_t UnionFindDisjointSet::merge(uint32_t x, uint32_t y) noexcept
{
    auto root_x = find(x), root_y = find(y);
    return parent[root_y] = parent[root_x];
}

void UnionFindDisjointSet::extract_disjoint_sets(stl_vector_mp<stl_vector_mp<uint32_t>>& disjoint_sets,
                                                 stl_vector_mp<uint32_t>&                index_map) noexcept
{
    const auto num_disjoint_set = parent.size();
    index_map.resize(num_disjoint_set, INVALID_INDEX);
    uint32_t counter{};
    // Assign each roots a unique index.
    for (uint32_t i = 0; i < num_disjoint_set; i++) {
        const auto root = find(i);
        if (root == i) {
            index_map[i] = counter;
            counter++;
        }
    }

    // Assign each element to its corresponding disjoint set.
    for (uint32_t i = 0; i < num_disjoint_set; i++) {
        const auto root = find(i);
        ROBUST_ASSERT(index_map[root] != INVALID_INDEX);
        index_map[i] = index_map[root];
    }

    disjoint_sets.resize(counter);
    for (uint32_t i = 0; i < num_disjoint_set; i++) {
        const auto index = index_map[i];
        disjoint_sets[index].emplace_back(i);
    }
}