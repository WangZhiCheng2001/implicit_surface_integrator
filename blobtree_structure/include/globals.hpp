#pragma once

#include <vector>
#include <stack>

#include <tbb/tbb.h>

#include "blobtree.h"
#include "internal_structs.hpp"

struct blobtree_t {
    std::vector<node_t, tbb::tbb_allocator<node_t>>     nodes{};
    std::vector<uint32_t, tbb::tbb_allocator<uint32_t>> leaf_index{};
};

extern std::vector<blobtree_t, tbb::tbb_allocator<blobtree_t>>                  structures;
extern std::vector<aabb_t, tbb::tbb_allocator<aabb_t>>                          aabbs;
extern std::vector<primitive_node_t, tbb::tbb_allocator<primitive_node_t>>      primitives;
extern std::stack<uint32_t, std::deque<uint32_t, tbb::tbb_allocator<uint32_t>>> free_structure_list;