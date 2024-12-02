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

#include <cstdint>
#include <cstddef>

namespace detail
{

/// \brief Select any of uint8_t, uint16_t, uint32_t or
/// unsigned long long. Result depends on N and on provision
/// of these types by compiler.
template <size_t N>
struct select_base {
#ifdef INT8_MIN
    enum : bool { has_int8 = true };

    using UI8 = uint8_t;
#else
    enum : bool { has_int8 = false };

    using UI8 = void;
#endif
#ifdef INT16_MIN
    enum : bool { has_int16 = true };

    using UI16 = uint16_t;
#else
    enum : bool { has_int16 = false };

    using UI16 = void;
#endif
#ifdef INT32_MIN
    enum : bool { has_int32 = true };

    using UI32 = uint32_t;
#else
    enum : bool { has_int32 = false };

    using UI32 = void;
#endif

    using type = std::conditional_t<has_int8 && (N <= 8),
                                    UI8,
                                    std::conditional_t<has_int16 && (N <= 16),
                                                       UI16,
                                                       std::conditional_t<has_int32 && (N <= 32), UI32, unsigned long long> > >;
}; // struct select_base

template <size_t N>
using select_base_t = typename select_base<N>::type;


} // namespace detail