#pragma once

#include <implicit_arrangement.hpp>

struct UnionFindDisjointSet {
    UnionFindDisjointSet() noexcept = default;
    UnionFindDisjointSet(uint32_t size) noexcept;

    void init(uint32_t size) noexcept;

    int32_t find(uint32_t x) noexcept;
    int32_t merge(uint32_t x, uint32_t y) noexcept;

    void extract_disjoint_sets(stl_vector_mp<stl_vector_mp<uint32_t>>& disjoint_sets,
                               stl_vector_mp<uint32_t>&                index_map) noexcept;

    stl_vector_mp<int32_t> parent{};
};