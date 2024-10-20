#pragma once

#include <container/small_vector.hpp>

struct UnionFindDisjointSet {
    UnionFindDisjointSet() noexcept = default;
    UnionFindDisjointSet(uint32_t size) noexcept;

    void init(uint32_t size) noexcept;

    int32_t find(uint32_t x) noexcept;
    int32_t merge(uint32_t x, uint32_t y) noexcept;

    void extract_disjoint_sets(uint32_t** disjoint_sets,
                               uint32_t*  index_map,
                               uint32_t*  disjoint_set_elem_count,
                               uint32_t&  num_disjoint_set) noexcept;

    small_vector_mp<int32_t> parent{};
};