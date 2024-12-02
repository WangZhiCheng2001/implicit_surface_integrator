// BITSET2
//
//  Copyright Claas Bontus
//
//  Use, modification and distribution is subject to the
//  Boost Software License, Version 1.0. (See accompanying
//  file LICENSE.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)
//
// Project home: https://github.com/ClaasBontus/bit_set
//


#pragma once

#include <bitset>
#include <array>
#include <stdexcept>
#include <string>
#include <functional>
#include <type_traits>


#include "detail/static_bitset/select_base_t.hpp"
#include "detail/static_bitset/hash.hpp"
#include "detail/static_bitset/array_funcs.hpp"
#include "detail/static_bitset/array_add.hpp"
#include "detail/static_bitset/array_ops.hpp"
#include "detail/static_bitset/array_complement2.hpp"
#include "detail/static_bitset/array2array.hpp"
#include "detail/static_bitset_base.hpp"


template <size_t N, class T = detail::select_base_t<N>, class Enabled = void>
class bit_set;

template <size_t N, class T>
class bit_set<N, T, typename std::enable_if<detail::is_unsgnd_int<T>::value>::type> : public detail::__bitset_base<N, T>
{
    enum : size_t { base_t_n_bits = detail::__bitset_base<N, T>::base_t_n_bits };

public:
    using array_t  = typename detail::__bitset_base<N, T>::array_t;
    using ULLONG_t = typename detail::__bitset_base<N, T>::ULLONG_t;
    using LRGST_t  = typename detail::__bitset_base<N, T>::LRGST_t;
    using base_t   = T;
    using detail::__bitset_base<N, T>::n_array;

    enum : size_t { npos = detail::h_types<T>::npos };

    class reference
    {
        friend class bit_set;

        reference() noexcept {}

        constexpr reference(bit_set<N, T> *ptr, size_t bit) noexcept : m_ptr(ptr), m_bit(bit) {}

        bit_set<N, T> *m_ptr = nullptr;
        size_t         m_bit;

    public:
        constexpr reference &operator=(bool x) noexcept
        {
            m_ptr->set_noexcept(m_bit, x);
            return *this;
        }

        constexpr reference &operator=(reference const &r) noexcept
        {
            m_ptr->set_noexcept(m_bit, bool(r));
            return *this;
        }

        constexpr reference &flip() noexcept
        {
            m_ptr->flip_noexcept(m_bit);
            return *this;
        }

        constexpr operator bool() const noexcept { return m_ptr->test_noexcept(m_bit); }

        constexpr bool operator~() const noexcept { return !bool(*this); }
    }; // class reference

    /* ------------------------------------------------------------- */
    constexpr bit_set() noexcept : detail::__bitset_base<N, T>() {}

    constexpr bit_set(bit_set const &) noexcept = default;

    constexpr bit_set(bit_set &&) noexcept = default;

    constexpr bit_set &operator=(bit_set const &) noexcept = default;

    constexpr bit_set &operator=(bit_set &&) noexcept = default;

    explicit bit_set(const std::bitset<N> &bs) noexcept : detail::__bitset_base<N, T>(bs) {}

    explicit constexpr bit_set(LRGST_t v) noexcept : detail::__bitset_base<N, T>(v) {}

    template <size_t n, class Tsrc>
    explicit constexpr bit_set(std::array<Tsrc, n> const &value) noexcept : detail::__bitset_base<N, T>(value)
    {
    }

    template <class CharT, class Traits, class Alloc>
    explicit bit_set(
        std::basic_string<CharT, Traits, Alloc> const              &str,
        typename std::basic_string<CharT, Traits, Alloc>::size_type pos  = 0,
        typename std::basic_string<CharT, Traits, Alloc>::size_type n    = std::basic_string<CharT, Traits, Alloc>::npos,
        CharT                                                       zero = CharT('0'),
        CharT                                                       one  = CharT('1'))
        : detail::__bitset_base<N, T>(str, pos, n, zero, one)
    {
    }

    template <class CharT>
    explicit bit_set(const CharT                                 *str,
                     typename std::basic_string<CharT>::size_type n    = std::basic_string<CharT>::npos,
                     CharT                                        zero = CharT('0'),
                     CharT                                        one  = CharT('1'))
        : detail::__bitset_base<N, T>(
            n == std::basic_string<CharT>::npos ? std::basic_string<CharT>(str) : std::basic_string<CharT>(str, n),
            0,
            n,
            zero,
            one)
    {
    }

    /* ------------------------------------------------------------- */


    //****************************************************

    /// Bitwise NOT
    constexpr bit_set operator~() const noexcept { return bit_set(detail::array_ops<N, T>(0).flip(this->data())); }

    constexpr bool operator[](size_t bit) const noexcept { return detail::__bitset_base<N, T>::operator[](bit); }

    constexpr reference operator[](size_t bit) noexcept { return reference(this, bit); }

    constexpr bit_set &operator<<=(size_t n_shift) noexcept
    {
        detail::array_ops<N, T>(n_shift).shift_left_assgn(this->get_data());
        return *this;
    }

    /// Shift left
    friend constexpr bit_set operator<<(bit_set const &bs, size_t n_shift) noexcept
    {
        return bit_set<N, T>(detail::array_ops<N, T>(n_shift).shift_left(bs.data()));
    }

    /// Stream output
    template <class CharT, class Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(std::basic_ostream<CharT, Traits> &os, bit_set const &x)
    {
        for (size_t ct = N; ct > 0;) {
            --ct;
            os << (x[ct] ? "1" : "0");
        }
        return os;
    }

    constexpr bit_set &operator>>=(size_t n_shift) noexcept
    {
        detail::array_ops<N, T>(n_shift).shift_right_assgn(this->get_data());
        return *this;
    }

    /// Shift right
    friend constexpr bit_set operator>>(bit_set const &bs, size_t n_shift) noexcept
    {
        return bit_set<N, T>(detail::array_ops<N, T>(n_shift).shift_right(bs.data()));
    }

    /// Stream input
    template <class CharT, class Traits>
    friend std::basic_istream<CharT, Traits> &operator>>(std::basic_istream<CharT, Traits> &is, bit_set &x)
    {
        std::bitset<N> bs;
        is >> bs;
        x = bit_set<N, T>(bs);
        return is;
    }

    constexpr bit_set &rotate_left(size_t n_rot) noexcept
    {
        this->get_data() = detail::array_ops<N, T>(n_rot).rotate_left(this->data());
        return *this;
    }

    constexpr bit_set &rotate_right(size_t n_rot) noexcept
    {
        this->get_data() = detail::array_ops<N, T>(N - (n_rot % N)).rotate_left(this->data());
        return *this;
    }

    constexpr bit_set &reverse() noexcept
    {
        this->get_data() = detail::array_ops<N, T>(0).reverse(this->data());
        return *this;
    }

    /// Computes two's complement
    constexpr bit_set &complement2() noexcept
    {
        detail::array_complement2<N, T>().comp2_assgn(this->get_data());
        return *this;
    }

    constexpr bit_set &operator+=(bit_set const &bs2) noexcept
    {
        detail::array_add<N, T>().add_assgn(this->get_data(), bs2.data());
        return *this;
    }

    friend constexpr bit_set operator+(bit_set const &bs1, bit_set const &bs2) noexcept
    {
        return bit_set<N, T>(detail::array_add<N, T>().add(bs1.data(), bs2.data()));
    }

    constexpr bit_set &operator++() noexcept
    {
        detail::array_ops<N, T>(0).increment(this->get_data());
        return *this;
    }

    constexpr bit_set operator++(int) noexcept
    {
        bit_set tmp(*this);
        operator++();
        return tmp;
    }

    constexpr bit_set &operator--() noexcept
    {
        detail::array_ops<N, T>(0).decrement(this->get_data());
        return *this;
    }

    constexpr bit_set operator--(int) noexcept
    {
        bit_set tmp(*this);
        operator--();
        return tmp;
    }

    constexpr bit_set &operator|=(bit_set const &v2) noexcept
    {
        detail::array_funcs<bit_set::n_array, T>().bitwise_or_assgn(this->get_data(), v2.data());
        return *this;
    }

    friend constexpr bit_set operator|(bit_set const &bs1, bit_set const &bs2) noexcept
    {
        return bit_set<N, T>(detail::array_funcs<bit_set::n_array, T>().bitwise_or(bs1.data(), bs2.data()));
    }

    constexpr bit_set &operator&=(bit_set const &v2) noexcept
    {
        detail::array_funcs<bit_set::n_array, T>().bitwise_and_assgn(this->get_data(), v2.data());
        return *this;
    }

    friend constexpr bit_set operator&(bit_set const &bs1, bit_set const &bs2) noexcept
    {
        return bit_set<N, T>(detail::array_funcs<bit_set::n_array, T>().bitwise_and(bs1.data(), bs2.data()));
    }

    constexpr bit_set &operator^=(bit_set const &v2) noexcept
    {
        detail::array_funcs<bit_set::n_array, T>().bitwise_xor_assgn(this->get_data(), v2.data());
        return *this;
    }

    friend constexpr bit_set operator^(bit_set const &bs1, bit_set const &bs2) noexcept
    {
        return bit_set<N, T>(detail::array_funcs<bit_set::n_array, T>().bitwise_xor(bs1.data(), bs2.data()));
    }

    /// Computes the set difference, i.e. *this &= ~v2
    constexpr bit_set &difference(bit_set const &v2) noexcept
    {
        detail::array_funcs<bit_set::n_array, T>().bitwise_setdiff_assgn(this->get_data(), v2.data());
        return *this;
    }

    constexpr bit_set &set() noexcept
    {
        detail::__bitset_base<N, T>::set();
        return *this;
    }

    constexpr bit_set &set(size_t bit, bool value = true)
    {
        detail::__bitset_base<N, T>::set(bit, value);
        return *this;
    }

    constexpr bit_set &reset() noexcept
    {
        detail::__bitset_base<N, T>::reset();
        return *this;
    }

    constexpr bit_set &reset(size_t bit)
    {
        if (bit >= N) throw std::out_of_range("bit_set: reset out of range");
        return set(bit, false);
    }

    /// \brief Sets the specified bit if value==true,
    /// clears it otherwise. Returns the previous state of the bit.
    constexpr bool test_set(size_t bit, bool value = true) { return detail::__bitset_base<N, T>::test_set(bit, value); }

    constexpr bit_set &flip() noexcept
    {
        detail::__bitset_base<N, T>::flip();
        return *this;
    }

    constexpr bit_set &flip(size_t bit)
    {
        detail::__bitset_base<N, T>::flip(bit);
        return *this;
    }

    constexpr std::size_t size() const noexcept { return N; }

    template <class CharT = char, class Traits = std::char_traits<CharT>, class Allocator = std::allocator<CharT> >
    std::basic_string<CharT, Traits, Allocator> to_string(CharT zero = CharT('0'), CharT one = CharT('1')) const
    {
        std::basic_string<CharT, Traits, Allocator> ret_val;
        ret_val.reserve(N);
        for (size_t ct = N; ct > 0;) {
            --ct;
            ret_val += this->operator[](ct) ? one : zero;
        }
        return ret_val;
    } // to_string
};    // class bit_set

template <size_t N, class T>
constexpr bit_set<N, T> rotate_left(bit_set<N, T> const &bs, size_t n_rot) noexcept
{
    return bit_set<N, T>(detail::array_ops<N, T>(n_rot).rotate_left(bs.data()));
}

template <size_t N, class T>
constexpr bit_set<N, T> rotate_right(bit_set<N, T> const &bs, size_t n_rot) noexcept
{
    return bit_set<N, T>(detail::array_ops<N, T>(N - (n_rot % N)).rotate_left(bs.data()));
}

/// Computes the set difference, i.e. bs1 & ~bs2
template <size_t N, class T>
constexpr bit_set<N, T> difference(bit_set<N, T> const &bs1, bit_set<N, T> const &bs2) noexcept
{
    return bit_set<N, T>(detail::array_funcs<bit_set<N, T>::n_array, T>().bitwise_setdiff(bs1.data(), bs2.data()));
}

/// Returns bs with bits reversed
template <size_t N, class T>
constexpr bit_set<N, T> reverse(bit_set<N, T> const &bs) noexcept
{
    return bit_set<N, T>(detail::array_ops<N, T>(0).reverse(bs.data()));
}

/// Computes the two's complement
template <size_t N, class T>
constexpr bit_set<N, T> complement2(bit_set<N, T> const &bs) noexcept
{
    return bit_set<N, T>(detail::array_complement2<N, T>().comp2(bs.data()));
}

/// Half the sum of bs1 and bs2. No overflow occurs.
template <size_t N, class T>
constexpr bit_set<N, T> midpoint(bit_set<N, T> const &bs1, bit_set<N, T> const &bs2, bool round_down = false) noexcept
{
    return bit_set<N, T>(detail::array_add<N, T>().midpoint(bs1.data(), bs2.data(), round_down));
}

/// Converts an M-bit bit_set to an N-bit bit_set.
template <size_t N, class T1, size_t M, class T2>
constexpr bit_set<N, T1> convert_to(bit_set<M, T2> const &bs) noexcept
{
    using a2a = detail::array2array<bit_set<N, T1>::n_array, bit_set<M, T2>::n_array, T1, T2>;
    return bit_set<N, T1>(a2a()(detail::bit_chars<N, T1>::hgh_bit_pattern, bs.data()));
}

/// Converts an M-bit bit_set to an N-bit bit_set.
template <size_t N, size_t M, class T>
constexpr bit_set<N, T> convert_to(bit_set<M, T> const &bs) noexcept
{
    return bit_set<N, T>(bs.data());
}

/// \brief Returns true if f returns true for each pair
/// of base_t=T values in bs1 and bs2. f should be a binary function
/// taking two base_t values and returning bool.
/// zip_fold_and does short circuit if possible.
template <size_t N, class F, class T>
constexpr bool zip_fold_and(bit_set<N, T> const &bs1, bit_set<N, T> const &bs2, F f) noexcept(noexcept(f(T(0), T(0))))
{
    return detail::array_funcs<bit_set<N, T>::n_array, T>().zip_fold_and(bs1.data(), bs2.data(), f);
}

/// \brief Returns true if f returns true for at least one pair
/// of base_t=T values in bs1 and bs2. f should be a binary function
/// taking two base_t values and returning bool.
/// zip_fold_or does short circuit if possible.
template <size_t N, class F, class T>
constexpr bool zip_fold_or(bit_set<N, T> const &bs1, bit_set<N, T> const &bs2, F f) noexcept(noexcept(f(T(0), T(0))))
{
    return detail::array_funcs<bit_set<N, T>::n_array, T>().zip_fold_or(bs1.data(), bs2.data(), f);
}

namespace std
{
template <size_t N, class T>
struct hash<bit_set<N, T> > {
private:
    enum : size_t { n_array = detail::__bitset_base<N, T>::n_array };

    detail::hash_impl<n_array, T> m_func;

public:
    using argument_type = bit_set<N, T>;
    using result_type   = typename detail::hash_impl<n_array, T>::result_type;

    result_type operator()(argument_type const &bs) const { return m_func(bs.data()); }
}; // struct hash

} // namespace std