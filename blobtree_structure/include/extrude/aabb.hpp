#pragma once
#include "vec.hpp"
#include "common.hpp"

class AABB
{
public:
    Vec3 min;
    Vec3 max;

    AABB() : min(Vec3(INFINITY, INFINITY, INFINITY)), max(Vec3(-INFINITY, -INFINITY, -INFINITY)) {}

    AABB(const Vec3 &minPoint, const Vec3 &maxPoint) : min(minPoint), max(maxPoint) {}

    void extend(const Vec3 &point)
    {
        min = minValue(min, point);
        max = maxValue(max, point);
    }

    void extend(const AABB &other)
    {
        min = minValue(min, other.min);
        max = maxValue(max, other.max);
    }

    void expand(real s = EPS_AABB_EXTENSION)
    {
        min = min - s;
        max = max + s;
    }

    void expand(const Vec3 &s)
    {
        min = min - s;
        max = max + s;
    }

    [[nodiscard]] Vec3 center() const { return (min + max) * HALF; }

    [[nodiscard]] Vec3 halfSize() const { return (max - min) * HALF; }

    void move(const Vec3 &v)
    {
        min = min + v;
        max = max + v;
    }

private:
    static Vec3 minValue(const Vec3 &a, const Vec3 &b)
    {
        return Vec3{std::min(a.x(), b.x()), std::min(a.y(), b.y()), std::min(a.z(), b.z())};
    }

    static Vec3 maxValue(const Vec3 &a, const Vec3 &b)
    {
        return Vec3{std::max(a.x(), b.x()), std::max(a.y(), b.y()), std::max(a.z(), b.z())};
    }
};