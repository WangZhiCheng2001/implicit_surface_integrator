#pragma once

#include "detail/dynamic_bitset_base.hpp"

#include "small_vector.hpp"

template <typename T = unsigned long long>
using dynamic_bitset = detail::dynamic_bitset<detail::stl_vector_bind<>::template type, T>;

template <typename T = unsigned long long>
using dynamic_bitset_mp = detail::dynamic_bitset<detail::stl_vector_bind<tbb::tbb_allocator>::template type, T>;
// using dynamic_bitset_mp = detail::dynamic_bitset<detail::stl_vector_bind<ScalableMemoryPoolAllocator>::template type, T>;

template <typename T = unsigned long long, std::size_t N = 1>
using small_dynamic_bitset = detail::dynamic_bitset<detail::small_vector_bind<N>::template type, T>;

template <typename T = unsigned long long, std::size_t N = 1>
using small_dynamic_bitset_mp = detail::dynamic_bitset<detail::small_vector_mp_bind<N>::template type, T>;