#pragma once

#include "common.hpp"
#include "real.hpp"
#include <cmath>
#include <array>
#include <assert.h>
#include <cstddef>
#include <ostream>
#include <vector>
#include <stdexcept>

template <typename Derived, size_t N>
class VecBase
{
public:
    std::array<real, N> data;

    VecBase() { data.fill(0); }

    VecBase(std::initializer_list<real> values) { std::copy(values.begin(), values.end(), data.begin()); }

    VecBase(real v) : data{v} {}

    VecBase(const VecBase &v) : data(v.data) {}

    template <typename... Args>
    explicit VecBase(Args... args) : data{static_cast<real>(args)...}
    {
        static_assert(sizeof...(args) == N, "Argument count must match vector size.");
    }

    VecBase &operator=(VecBase v)
    {
        *this = std::move(v);
        return *this;
    }

    real &operator[](size_t index)
    {
        if (index >= N) throw std::out_of_range("Index out of range");
        return data[index];
    }

    const real &operator[](size_t index) const
    {
        if (index >= N) throw std::out_of_range("Index out of range");
        return data[index];
    }

    Derived operator+(const VecBase &v) const
    {
        Derived result;
        for (size_t i = 0; i < N; ++i) { result[i] = data[i] + v[i]; }
        return result;
    }

    Derived operator-(const VecBase &v) const
    {
        Derived result;
        for (size_t i = 0; i < N; ++i) { result[i] = data[i] - v[i]; }
        return result;
    }

    Derived operator*(real s) const
    {
        Derived result;
        for (size_t i = 0; i < N; ++i) { result[i] = data[i] * s; }
        return result;
    }

    friend Derived operator*(real s, const VecBase &v)
    {
        Derived result;
        for (size_t i = 0; i < N; ++i) { result[i] = s * v[i]; }
        return result;
    }

    Derived operator/(real s) const
    {
        Derived result;
        for (size_t i = 0; i < N; ++i) { result[i] = data[i] / s; }
        return result;
    }

    real dot(const VecBase &v) const
    {
        real sum = 0;
        for (size_t i = 0; i < N; ++i) { sum += data[i] * v[i]; }
        return sum;
    }

    real norm() const { return std::sqrt(dot(*this)); }

    real l2() const { return dot(*this); }

    Derived normalize() const { return static_cast<Derived>(*this / norm()); }

    Derived reflect(const Derived &n) const { return static_cast<Derived>(*this - n * 2 * dot(n)); }

    bool operator==(const VecBase &v) const
    {
        for (size_t i = 0; i < N; ++i) {
            if (!isEqual(data[i], v[i])) return false;
        }
        return true;
    }

    friend std::ostream &operator<<(std::ostream &os, const VecBase &vec)
    {
        os << "Vec" << N << "(";
        for (size_t i = 0; i < N; ++i) {
            os << vec[i];
            if (i < N - 1) os << ", ";
        }
        os << ")";
        return os;
    }

    Derived operator-() const { return static_cast<Derived>(*this * -1); }
};

// Vec<2> 特化
class Vec2 : public VecBase<Vec2, 2>
{
public:
    Vec2() : VecBase() {}

    Vec2(real x, real y) : VecBase(x, y) {}

    Vec2(const Vec2 &v) : VecBase(v.x(), v.y()) {}

    Vec2(real v) : VecBase(v) {}

    Vec2 &operator=(const Vec2 &v)
    {
        data = v.data;
        return *this;
    }

    real &x() { return data[0]; }

    real &y() { return data[1]; }

    [[nodiscard]] const real &x() const { return data[0]; }

    [[nodiscard]] const real &y() const { return data[1]; }

    real &u() { return data[0]; }

    real &v() { return data[1]; }

    [[nodiscard]] const real &u() const { return data[0]; }

    [[nodiscard]] const real &v() const { return data[1]; }

    [[nodiscard]] real cross(const Vec2 &v) const { return x() * v.y() - v.x() * y(); }
};

class Vec3 : public VecBase<Vec3, 3>
{
public:
    Vec3() : VecBase() {}

    Vec3(real x, real y, real z) : VecBase(x, y, z) {}

    Vec3(const Vec3 &v) : VecBase(v.x(), v.y(), v.z()) {}

    Vec3(real v) : VecBase(v) {}

    Vec3 &operator=(const Vec3 &v)
    {
        data = v.data;
        return *this;
    }

    real &x() { return data[0]; }

    real &y() { return data[1]; }

    real &z() { return data[2]; }

    [[nodiscard]] real x() const { return data[0]; }

    [[nodiscard]] real y() const { return data[1]; }

    [[nodiscard]] real z() const { return data[2]; }

    real &u() { return data[0]; }

    real &v() { return data[1]; }

    real &w() { return data[2]; }

    [[nodiscard]] real u() const { return data[0]; }

    [[nodiscard]] real v() const { return data[1]; }

    [[nodiscard]] real w() const { return data[2]; }

    [[nodiscard]] Vec3 cross(const Vec3 &v) const
    {
        return {y() * v.z() - z() * v.y(), z() * v.x() - x() * v.z(), x() * v.y() - y() * v.x()};
    }

    [[nodiscard]] bool isParallel(const Vec3 &v) const { return cross(v) == 0; }
};

using Pt3Array = std::vector<Vec3>;
using Pt2Array = std::vector<Vec2>;

// class Vec3 {
// public:
//     real x, y, z;
//     Vec3(): x(0), y(0), z(0) {}
//     Vec3(real x, real y, real z): x(x), y(y), z(z) {}
//     Vec3(const Vec3& v): x(v.x), y(v.y), z(v.z) {}
//     Vec3& operator=(const Vec3& v) {
//         x = v.x;
//         y = v.y;
//         z = v.z;
//         return *this;
//     }
//     Vec3 operator+(const Vec3& v) const {
//         return Vec3(x + v.x, y + v.y, z + v.z);
//     }
//     Vec3 operator-(const Vec3& v) const {
//         return Vec3(x - v.x, y - v.y, z - v.z);
//     }
//     Vec3 operator*(real s) const {
//         return Vec3(x * s, y * s, z * s);
//     }
//     Vec3 operator*(const Vec3& v) const {
//         return Vec3(x * v.x, y * v.y, z * v.z);
//     }
//     Vec3 operator-() const {
//         return Vec3(-x, -y, -z);
//     }
//     friend Vec3 operator*(real s, const Vec3& v) {
//         return Vec3(s * v.x, s * v.y, s * v.z);
//     }
//     Vec3 operator/(real s) const {
//         return Vec3(x / s, y / s, z / s);
//     }
//     real dot(const Vec3& v) const {
//         return x * v.x + y * v.y + z * v.z;
//     }
//     Vec3 cross(const Vec3& v) const {
//         return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
//     }
//     real norm() const {
//         return sqrt(x * x + y * y + z * z);
//     }
//     Vec3 normalize() const {
//         return *this / norm();
//     }
//     Vec3 reflect(const Vec3& n) const {
//         return *this - n * 2 * dot(n);
//     }
// };
