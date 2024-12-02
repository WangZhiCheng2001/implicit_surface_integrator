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

#include <limits>
#include <climits>
#include <cstddef>
#include <type_traits>

#include <countl_zero.hpp>

namespace detail
{
/// https://graphics.stanford.edu/~seander/bithacks.html#ZerosOnRightBinSearch
template <class T>
struct index_msb_set {
    enum : size_t { npos = std::numeric_limits<size_t>::max(), n_bits = sizeof(T) * CHAR_BIT };

    constexpr index_msb_set() noexcept
    {
        static_assert((n_bits & (n_bits - 1)) == 0, "Number of bits in data type is not a power of 2");
    }

    constexpr size_t impl(T val) const noexcept
    {
        T const fnl_bit = T(T(1) << (n_bits - 1));
        if (T(val & fnl_bit) == fnl_bit) return n_bits - 1;

        size_t c      = 1;
        size_t sh_lft = n_bits >> 1;
        T      pttrn  = T(T(~T(0)) << sh_lft);

        while (sh_lft >= 2) {
            if (T(val & pttrn) == T(0)) {
                c    += sh_lft;
                val <<= sh_lft;
            }
            sh_lft >>= 1;
            pttrn  <<= sh_lft;
        }
        if (T(val & fnl_bit) == fnl_bit) --c;

        return n_bits - 1 - c;
    } // impl

    /// \brief Returns index of last (most significant) bit set in val.
    /// Returns npos if all bits are zero.
    constexpr size_t operator()(T val) const noexcept
    {
        if (T(0) == val) return npos;
#ifdef __SIZEOF_INT128__
        if constexpr (std::is_same_v<T, unsigned __int128>)
            return impl(val);
        else
#endif
            return n_bits - 1 - __countl_zero(val);
    }
}; // struct index_msb_set
} // namespace detail