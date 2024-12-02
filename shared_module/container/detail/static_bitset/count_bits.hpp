// BITSET2
//
//  Copyright Claas Bontus
//
//  Use, modification and distribution is subject to the
//  Boost Software License, Version 1.0. (See accompanying
//  file LICENSE.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)
//
// Project home: https://github.com/ClaasBontus/bitset2
//

#pragma once

#include <utils/popcount.hpp>

#include "h_types.hpp"

namespace detail
{

/// Returns the number of bits set in val
template <class T>
constexpr inline size_t count_bits(T val, size_t count = 0) noexcept
{
#ifdef __SIZEOF_INT128__
    if constexpr (!std::is_same_v<T, unsigned __int128>)
#endif
        return popcnt(&val, sizeof(T));
}

} // namespace detail