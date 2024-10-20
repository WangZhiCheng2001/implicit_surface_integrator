#pragma once

#include <vector>

#include "detail/small_vector_base.hpp"
#include "../memory/memory_pool.hpp"

namespace detail
{
template <template <typename> class Allocator = std::allocator>
struct stl_vector_bind{
    template <typename T>
    using type = std::vector<T, Allocator<T>>;
};
} // namespace detail

template <typename T, std::size_t N = 16>
using small_vector = detail::small_vector_base<detail::stl_vector_bind<>::template type, T, N>;

template <typename T, std::size_t N = 16>
using small_vector_mp = detail::small_vector_base<detail::stl_vector_bind<ScalableMemoryPoolAllocator>::template type, T, N>;

namespace detail
{
template <size_t N>
struct small_vector_bind {
    template <typename T>
    using type = small_vector<T, N>;
};

template <size_t N>
struct small_vector_mp_bind {
    template <typename T>
    using type = small_vector_mp<T, N>;
};
} // namespace detail
