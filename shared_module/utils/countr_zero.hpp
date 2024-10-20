#pragma once

// from: https://gist.github.com/pps83/3210a2f980fd02bb2ba2e5a1fc4a2ef0

// Note, bsf/bsr are used by default.
// Enable /arch:AVX2 compilation for better optimizations

#include <limits>
#if defined(_MSC_VER) && !defined(__clang__)
#include <intrin.h>

static __forceinline int __builtin_ctz(unsigned x)
{
#if defined(_M_ARM) || defined(_M_ARM64) || defined(_M_HYBRID_X86_ARM64) || defined(_M_ARM64EC)
    return (int)_CountTrailingZeros(x);
#elif defined(__AVX2__) || defined(__BMI__)
    return (int)_tzcnt_u32(x);
#else
    unsigned long r;
    _BitScanForward(&r, x);
    return (int)r;
#endif
}

static __forceinline int __builtin_ctzll(unsigned long long x)
{
#if defined(_M_ARM) || defined(_M_ARM64) || defined(_M_HYBRID_X86_ARM64) || defined(_M_ARM64EC)
    return (int)_CountTrailingZeros64(x);
#elif defined(_WIN64)
#if defined(__AVX2__) || defined(__BMI__)
    return (int)_tzcnt_u64(x);
#else
    unsigned long r;
    _BitScanForward64(&r, x);
    return (int)r;
#endif
#else
    int l = __builtin_ctz((unsigned)x);
    int h = __builtin_ctz((unsigned)(x >> 32)) + 32;
    return !!((unsigned)x) ? l : h;
#endif
}

static __forceinline int __builtin_ctzl(unsigned long x)
{
    return sizeof(x) == 8 ? __builtin_ctzll(x) : __builtin_ctz((unsigned)x);
}
#endif // defined(_MSC_VER) && !defined(__clang__)

template <typename _Tp>
constexpr int __countr_zero(_Tp __x) noexcept
{
    constexpr auto _Nd = std::numeric_limits<_Tp>::digits;

    if (__x == 0) return _Nd;

    constexpr auto _Nd_ull = std::numeric_limits<unsigned long long>::digits;
    constexpr auto _Nd_ul  = std::numeric_limits<unsigned long>::digits;
    constexpr auto _Nd_u   = std::numeric_limits<unsigned>::digits;

    if constexpr (_Nd <= _Nd_u)
        return __builtin_ctz(__x);
    else if constexpr (_Nd <= _Nd_ul)
        return __builtin_ctzl(__x);
    else if constexpr (_Nd <= _Nd_ull)
        return __builtin_ctzll(__x);
    else // (_Nd > _Nd_ull)
    {
        static_assert(_Nd <= (2 * _Nd_ull), "Maximum supported integer size is 128-bit");

        constexpr auto     __max_ull = std::numeric_limits<unsigned long long>::max();
        unsigned long long __low     = __x & __max_ull;
        if (__low != 0) return __builtin_ctzll(__low);
        unsigned long long __high = __x >> _Nd_ull;
        return __builtin_ctzll(__high) + _Nd_ull;
    }
}