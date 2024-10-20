#include <limits>

#include <range/v3/range.hpp>
#include <range/v3/view.hpp>

#include "union_find_disjoint_set.hpp"
#include "memory/memory_pool.hpp"
#include "robust_assert.hpp"

UnionFindDisjointSet::UnionFindDisjointSet(uint32_t size) noexcept { init(size); }

void UnionFindDisjointSet::init(uint32_t size) noexcept { parent.resize(size, -1); }

int32_t UnionFindDisjointSet::find(uint32_t x) noexcept
{
    while (parent[x] >= 0) {
        parent[x] = parent[parent[x]];
        x         = parent[x];
    }
    return x;
}

int32_t UnionFindDisjointSet::merge(uint32_t x, uint32_t y) noexcept
{
    auto root_x = find(x), root_y = find(y);
    if (root_x != root_y) {
        if (parent[root_x] < parent[root_y])
            return parent[root_y] = root_x;
        else if (parent[root_x] > parent[root_y])
            return parent[root_x] = root_y;
        else {
            parent[root_x]--;
            return parent[root_y] = root_x;
        }
    }

    return -1;
}

void UnionFindDisjointSet::extract_disjoint_sets(uint32_t** disjoint_sets,
                                                 uint32_t*  index_map,
                                                 uint32_t*  disjoint_set_elem_count,
                                                 uint32_t&  num_disjoint_set) noexcept
{
    num_disjoint_set = static_cast<uint32_t>(parent.size());
    index_map = static_cast<uint32_t*>(ScalableMemoryPoolSingleton::instance().malloc(sizeof(uint32_t) * num_disjoint_set));
    memset(index_map, std::numeric_limits<uint32_t>::max(), sizeof(uint32_t) * num_disjoint_set);
    uint32_t counter{};

    // Assign each roots a unique index.
    for (uint32_t i = 0; i < num_disjoint_set; i++) {
        const auto root = find(i);
        if (root < 0) { index_map[i] = counter++; }
    }

    disjoint_sets = static_cast<uint32_t**>(ScalableMemoryPoolSingleton::instance().malloc(sizeof(uint32_t*) * counter));
    memset(disjoint_sets, 0, sizeof(uint32_t*) * counter);
    disjoint_set_elem_count =
        static_cast<uint32_t*>(ScalableMemoryPoolSingleton::instance().malloc(sizeof(uint32_t) * counter));
    memset(disjoint_set_elem_count, 0, sizeof(uint32_t) * counter);
    // Assign each element to its corresponding disjoint set.
    for (uint32_t i = 0; i < num_disjoint_set; i++) {
        const auto root = find(i);
        ROBUST_ASSERT(index_map[root] < 0);
        index_map[i] = index_map[root];
        disjoint_set_elem_count[index_map[i]]++;
    }

    // for (uint32_t i = 0; i < counter; i++)
    //     disjoint_sets[i] =
    //         static_cast<uint32_t*>(ScalableMemoryPoolSingleton::instance().malloc(sizeof(uint32_t) * disjoint_set_count[i]));

    // disjoint_set_count.assign(counter, 0);
    for (uint32_t i = 0; i < num_disjoint_set; i++) {
        const auto index = index_map[i];
        if (disjoint_sets[index] == nullptr) {
            disjoint_sets[index] = static_cast<uint32_t*>(
                ScalableMemoryPoolSingleton::instance().malloc(sizeof(uint32_t) * disjoint_set_elem_count[index]));
            disjoint_set_elem_count[index] = 0;
        }
        disjoint_sets[index][disjoint_set_elem_count[index]++] = i;
    }
}