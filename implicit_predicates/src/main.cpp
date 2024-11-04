#include <cmath>
#include <cassert>
#include <type_traits>

#include <implicit_predicates.hpp>

#include "internal/det2.cpp"
#include "internal/det3.cpp"
#include "internal/det4.cpp"
#include "internal/det5.cpp"
#include "internal/diff_det2.cpp"
#include "internal/diff_det2_one.cpp"
#include "internal/diff_det3.cpp"
#include "internal/diff_det3_one.cpp"
#include "internal/diff_det4.cpp"
#include "internal/diff_det4_one.cpp"

template <typename T>
orientation sign_of(T val)
{
    // Check for nan and inf.
    if constexpr (std::is_same<T, double>::value) assert(std::isfinite(val));

    if (val > 0)
        return orientation::positive;
    else if (val < 0)
        return orientation::negative;
    else
        return orientation::zero;
}

EXTERN_C_BEGIN

IP_API orientation orient1d(const double f0[2], const double f1[2])
{
    if (f0[0] == f0[1]) {
        // Function 0 is constant.
        return orientation::invalid;
    } else if (f0[1] < f0[0]) {
        return sign_of(det2(f0[0], f0[1], f1[0], f1[1]));
    } else {
        return sign_of(-det2(f0[0], f0[1], f1[0], f1[1]));
    }
}

IP_API orientation orient1d_nonrobust(const double f0[2], const double f1[2])
{
    if (f0[0] == f0[1]) {
        // Function 0 is constant.
        return orientation::invalid;
    } else if (f0[1] < f0[0]) {
        return sign_of(f0[0] * f1[1] - f0[1] * f1[0]);
    } else {
        return sign_of(-f0[0] * f1[1] + f0[1] * f1[0]);
    }
}

IP_API orientation orient2d(const double f0[3], const double f1[3], const double f2[3])
{
    // clang-format off
    const auto denominator = det3(
            f0[0], f0[1], f0[2],
            f1[0], f1[1], f1[2],
                1,     1,     1);
    const auto numerator = det3(
            f0[0], f0[1], f0[2],
            f1[0], f1[1], f1[2],
            f2[0], f2[1], f2[2]);
    // clang-format on

    if (denominator == 0) {
        return orientation::invalid;
    } else if (denominator > 0) {
        return sign_of(numerator);
    } else {
        return sign_of(-numerator);
    }
}

IP_API orientation orient2d_nonrobust(const double f0[3], const double f1[3], const double f2[3])
{
    auto det2 = [](double a, double b, double c, double d) { return a * d - b * c; };

    const auto d0 = det2(f0[1], f0[2], f1[1], f1[2]);
    const auto d1 = -det2(f0[0], f0[2], f1[0], f1[2]);
    const auto d2 = det2(f0[0], f0[1], f1[0], f1[1]);

    const auto denominator = d0 + d1 + d2;

    if (denominator == 0) {
        return orientation::invalid;
    } else if (denominator > 0) {
        return sign_of(f2[0] * d0 + f2[1] * d1 + f2[2] * d2);
    } else {
        return sign_of(-f2[0] * d0 - f2[1] * d1 - f2[2] * d2);
    }
}

IP_API orientation orient3d(const double f0[4], const double f1[4], const double f2[4], const double f3[4])
{
    // clang-format off
    const auto numerator = det4(
            f0[0], f0[1], f0[2], f0[3],
            f1[0], f1[1], f1[2], f1[3],
            f2[0], f2[1], f2[2], f2[3],
            f3[0], f3[1], f3[2], f3[3]);
    const auto denominator = det4(
            f0[0], f0[1], f0[2], f0[3],
            f1[0], f1[1], f1[2], f1[3],
            f2[0], f2[1], f2[2], f2[3],
                1,     1,     1,     1);
    // clang-format on
    if (denominator == 0) {
        return orientation::invalid;
    } else if (denominator > 0) {
        return sign_of(numerator);
    } else {
        return sign_of(-numerator);
    }
}

IP_API orientation orient3d_nonrobust(const double f0[4], const double f1[4], const double f2[4], const double f3[4])
{
    // clang-format off
    const auto d0 = -det3(
            f0[1], f0[2], f0[3],
            f1[1], f1[2], f1[3],
            f2[1], f2[2], f2[3]);
    const auto d1 = det3(
            f0[0], f0[2], f0[3],
            f1[0], f1[2], f1[3],
            f2[0], f2[2], f2[3]);
    const auto d2 = -det3(
            f0[0], f0[1], f0[3],
            f1[0], f1[1], f1[3],
            f2[0], f2[1], f2[3]);
    const auto d3 = det3(
            f0[0], f0[1], f0[2],
            f1[0], f1[1], f1[2],
            f2[0], f2[1], f2[2]);
    // clang-format on

    const auto denominator = d0 + d1 + d2 + d3;

    if (denominator == 0) {
        return orientation::invalid;
    } else if (denominator > 0) {
        return sign_of(d0 * f3[0] + d1 * f3[1] + d2 * f3[2] + d3 * f3[3]);
    } else {
        return sign_of(-d0 * f3[0] - d1 * f3[1] - d2 * f3[2] - d3 * f3[3]);
    }
}

IP_API orientation orient4d(const double f0[5], const double f1[5], const double f2[5], const double f3[5], const double f4[5])
{
    // clang-format off
    const auto numerator = det5(
            f0[0], f0[1], f0[2], f0[3], f0[4],
            f1[0], f1[1], f1[2], f1[3], f1[4],
            f2[0], f2[1], f2[2], f2[3], f2[4],
            f3[0], f3[1], f3[2], f3[3], f3[4],
            f4[0], f4[1], f4[2], f4[3], f4[4]);
    const auto denominator = det5(
            f0[0], f0[1], f0[2], f0[3], f0[4],
            f1[0], f1[1], f1[2], f1[3], f1[4],
            f2[0], f2[1], f2[2], f2[3], f2[4],
            f3[0], f3[1], f3[2], f3[3], f3[4],
                1,     1,     1,     1,     1);
    // clang-format on
    if (denominator == 0) {
        return orientation::invalid;
    } else if (denominator > 0) {
        return sign_of(numerator);
    } else {
        return sign_of(-numerator);
    }
}

IP_API orientation
    orient4d_nonrobust(const double f0[5], const double f1[5], const double f2[5], const double f3[5], const double f4[5])
{
    // clang-format off
    const auto d0 = det4(
            f0[1], f0[2], f0[3], f0[4],
            f1[1], f1[2], f1[3], f1[4],
            f2[1], f2[2], f2[3], f2[4],
            f3[1], f3[2], f3[3], f3[4]);
    const auto d1 = -det4(
            f0[0], f0[2], f0[3], f0[4],
            f1[0], f1[2], f1[3], f1[4],
            f2[0], f2[2], f2[3], f2[4],
            f3[0], f3[2], f3[3], f3[4]);
    const auto d2 = det4(
            f0[0], f0[1], f0[3], f0[4],
            f1[0], f1[1], f1[3], f1[4],
            f2[0], f2[1], f2[3], f2[4],
            f3[0], f3[1], f3[3], f3[4]);
    const auto d3 = -det4(
            f0[0], f0[1], f0[2], f0[4],
            f1[0], f1[1], f1[2], f1[4],
            f2[0], f2[1], f2[2], f2[4],
            f3[0], f3[1], f3[2], f3[4]);
    const auto d4 = det4(
            f0[0], f0[1], f0[2], f0[3],
            f1[0], f1[1], f1[2], f1[3],
            f2[0], f2[1], f2[2], f2[3],
            f3[0], f3[1], f3[2], f3[3]);
    // clang-format on

    const auto denominator = d0 + d1 + d2 + d3 + d4;

    if (denominator == 0) {
        return orientation::invalid;
    } else if (denominator > 0) {
        return sign_of(d0 * f4[0] + d1 * f4[1] + d2 * f4[2] + d3 * f4[3] + d4 * f4[4]);
    } else {
        return sign_of(-d0 * f4[0] - d1 * f4[1] - d2 * f4[2] - d3 * f4[3] - d4 * f4[4]);
    }
}

EXTERN_C_END