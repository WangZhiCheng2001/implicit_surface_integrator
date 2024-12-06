#pragma once

#include "common.hpp"
#include "real.hpp"
#include "vec.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits.h>
#include <limits>
#include <vector>
#include "aabb.hpp"

// class ILineParam {
// public:
//     virtual ~ILineParam() = default;
// };
//
// class PolylineParam : ILineParam {
//     int segIdx;
//     real tOnSeg;
// };
//
// class PolynomialLineParam : ILineParam {
//     real t;
// };

struct ClosestDescOnLine {
    real t;
    real dis;

    ClosestDescOnLine(real _t, real _dis) : t(_t), dis(_dis) {}

    ClosestDescOnLine() : t(0), dis(std::numeric_limits<real>::max()) {}
};

struct ClosestDescOnProfile : public ClosestDescOnLine {
    int i; // line idx

    ClosestDescOnProfile(real _t, real _dis, int _i) : ClosestDescOnLine(_t, _dis), i(_i) {}

    ClosestDescOnProfile() : i(-1) {}
};

// vscode C++ override跳转插件
class ILine
{
public:
    int aaa;
    virtual ~ILine()                = default;
    ILine()                         = default;
    ILine(const ILine &)            = default;
    ILine &operator=(const ILine &) = default;
    ILine(ILine &&)                 = default;
    ILine &operator=(ILine &&)      = default;

    [[nodiscard]] virtual Vec3 eval(real t) const = 0;

    [[nodiscard]] virtual Vec3 der1(real t) const = 0;

    [[nodiscard]] virtual Vec3 der2(real t) const = 0;

    [[nodiscard]] virtual Vec3 tangent(real t) const = 0;

    [[nodiscard]] virtual Vec3 normal(real t, const Vec3 &tan = -1.) const = 0;

    [[nodiscard]] virtual ClosestDescOnLine getClosestParam(const Vec3 &p) const = 0;

    [[nodiscard]] virtual bool isEndParam(real t) const = 0;

    [[nodiscard]] virtual real startT() const = 0;
    [[nodiscard]] virtual real endT() const   = 0;

    [[nodiscard]] virtual AABB getAABB() const = 0;
};

inline static ClosestDescOnLine segPtDist(const Vec3 &p, const Vec3 &A, const Vec3 &B)
{
    Vec3 AB = B - A;
    Vec3 AP = p - A;
    real h  = std::clamp(AP.dot(AB) / AB.dot(AB), 0., 1.);
    return {h, (AP - AB * h).norm()};
}

inline static ClosestDescOnLine segPtDist(const Vec2 &p, const Vec2 &A, const Vec2 &B)
{
    Vec2 AB = B - A;
    Vec2 AP = p - A;
    real h  = std::clamp(AP.dot(AB) / AB.dot(AB), 0., 1.);
    return {h, (AP - AB * h).norm()};
}

template <typename VecType> // Vec2 or Vec3
struct CircularArc {
    VecType center;
    real    radius;
    real    theta;
    real    h = -1; // straight line for h <= 0.
    VecType u;
    VecType v;
    VecType inCircleDir;

    [[nodiscard]] PtBoundaryRelation inCircleCheck(const VecType &pt) const
    {
        real d = (pt - center).norm();
        return d < radius ? Inside : d > radius ? Outside : OnBoundary;
    }
};

;

struct AA {
    int a;
    int b;

    void print() const { std::cout << a << " " << b << std::endl; }
};

const real DISC_ARC_ANGLE = PI * 0.125;

class PolyLine : public ILine
{
public:
    PolyLine(Pt3Array points, std::vector<real> bugles, const Vec3 &refNormal, bool closed = false)
        : _points(std::move(points)), _bugles(std::move(bugles)), _closed(closed), _refNormal(refNormal.normalize())
    {
        assert(_points.size() >= 2);
        if (closed) {
            assert(_points.size() == _bugles.size());
        } else {
            assert(_points.size() - 1 == _bugles.size());
        }
        circularArcs.resize(_bugles.size());
        initSegInfo();
        for (size_t i = 0; i < _bugles.size(); ++i) {
            AABB tmp{_points[i], _points[i]};
            tmp.extend(AABB{_points[(i + 1) % _points.size()], _points[(i + 1) % _points.size()]});
            tmp.expand(circularArcs[i].radius - circularArcs[i].h);
            aabb.extend(tmp);
        }
    }

    [[nodiscard]] const Pt3Array &getPoints() const { return _points; }

    [[nodiscard]] const std::vector<real> &getBugles() const { return _bugles; }

    [[nodiscard]] const Vec3 &getRefNormal() const { return _refNormal; }

    [[nodiscard]] bool isClosed() const { return _closed; }

    [[nodiscard]] const std::vector<CircularArc<Vec3>> &getCircularArcs() const { return circularArcs; }

    [[nodiscard]] real startT() const override { return 0; };

    [[nodiscard]] real endT() const override { return static_cast<real>(_bugles.size()); };

protected:
    Pt3Array                       _points;
    std::vector<real>              _bugles;
    Vec3                           _refNormal;
    bool                           _closed;
    std::vector<CircularArc<Vec3>> circularArcs;

    AABB aabb;

public:
    void initSegInfo()
    {
        for (size_t i = 0; i < _bugles.size(); ++i) {
            initCircularArcInfo(_points[i], _points[(i + 1) % _points.size()], _bugles[i], _refNormal, circularArcs[i]);
        }
    }

    [[nodiscard]] Vec3 eval(real t) const override
    {
        // if (circularArcs.empty())
        //     initSegInfo();
        int seg = static_cast<int>(t);
        if (isEqual(_bugles[seg], 0)) {
            return _points[seg] + (_points[(seg + 1) % _points.size()] - _points[seg]) * (t - seg);
        }
        real        tOnSeg = t - seg;
        const auto &arc    = circularArcs[seg];
        real        phi    = tOnSeg * arc.theta;
        return arc.center + arc.radius * (arc.u * std::cos(phi) + arc.v * std::sin(phi));
    }

    [[nodiscard]] Vec3 der1(real t) const override
    {
        int seg = static_cast<int>(t);
        if (isEqual(_bugles[seg], 0)) { return _points[(seg + 1) % _points.size()] - _points[seg]; }
        real        tOnSeg = t - seg;
        const auto &arc    = circularArcs[seg];
        real        phi    = tOnSeg * arc.theta;
        return arc.radius * (arc.u * -std::sin(phi) + arc.v * std::cos(phi));
    }

    [[nodiscard]] Vec3 der2(real t) const override
    {
        int seg = static_cast<int>(t);
        if (isEqual(_bugles[seg], 0)) { int aaa = 1; }
        // assert(!isEqual(_bugles[seg], 0));
        real        tOnSeg = t - seg;
        const auto &arc    = circularArcs[seg];
        real        phi    = tOnSeg * arc.theta;
        return -arc.radius * (arc.u * std::cos(phi) + arc.v * std::cos(phi));
    }

    Vec3 tangent(real t) const override { return der1(t).normalize(); }

    // TODO: 试试https://www.jianshu.com/p/9e4877e3965e算出来的结果
    Vec3 normal(real t, [[maybe_unused]] const Vec3 &tan = -1.) const override
    {
        if (isEqual(_bugles[static_cast<int>(t)], 0)) { return -circularArcs[static_cast<int>(t)].inCircleDir; }
        // 只有对于圆弧这样的特殊曲线是这样
        return der2(t).normalize();
    }

    // ClosestDescOnSeg getClosestParam(const Vec3 &p) override {
    //     real closestDis = std::numeric_limits<real>::max();
    //     real closestParam;
    //     for (int i = 0; i < _bugles.size(); ++i) {
    //         const Vec3 &A = _points[i];
    //         const Vec3 &B = _points[(i + 1) % _points.size()];
    //         const auto &arc = circularArcs[i];
    //         real dis2Seg = segPtDist(p, A, B).dis;
    //         if (dis2Seg - arc.h > closestDis)
    //             continue;
    //         if ((A - p).norm() < closestDis) {
    //             closestDis = (A - p).norm();
    //             closestParam = i;
    //         }
    //         if ((B - p).norm() < closestDis) {
    //             closestDis = (B - p).norm();
    //             closestParam = i + 1;
    //         }
    //         int segInsertedCnt = arc.theta / DISC_ARC_ANGLE;
    //         for (int j = 0; j < segInsertedCnt; ++j) {
    //             real insertParam = i + j * DISC_ARC_ANGLE / arc.theta;
    //             const Vec3 insertPt = eval(insertParam);
    //             real dis2InsertPt = (p - insertPt).norm();
    //             if (dis2InsertPt < closestDis) {
    //                 closestDis = dis2InsertPt;
    //                 closestParam = insertParam;
    //             }
    //         }
    //     }
    //     // TODO: 为了鲁棒和精度，应该在每个可能最近的seg上做newton iteration
    //     int seg = static_cast<int>(closestParam);
    //     // Q = arc.center + arc.radius * (arc.u * std::cos(phi) + arc.v *
    //     // std::sin(phi)) d2 = (Q - p)^2
    //     Vec3 q = eval(closestParam);
    //     Vec3 qDer1 = der1(closestParam);
    //     Vec3 qDer2 = der2(closestParam);
    //     real lDer1 = (q - p).dot(qDer1);
    //     int iter = 0;
    //     while (fabs(lDer1) > std::numeric_limits<real>::epsilon() * 1e6) {
    //         closestParam -= lDer1 / (qDer1.dot(qDer1) + (q - p).dot(qDer2)); // -der1 / der2
    //         q = eval(closestParam);
    //         qDer1 = der1(closestParam);
    //         qDer2 = der2(closestParam);
    //         lDer1 = (q - p).dot(qDer1);
    //         printf("After iter %d, dL is %lf\n", iter, lDer1);
    //         if (closestParam < seg - std::numeric_limits<real>::epsilon()) {
    //             closestParam = seg;
    //             closestDis = (_points[seg] - p).norm();
    //             break;
    //         }
    //         if (closestParam > seg + 1 + std::numeric_limits<real>::epsilon()) {
    //             closestParam = seg + 1;
    //             closestDis = (_points[(seg + 1) % _points.size()] - p).norm();
    //             break;
    //         }
    //         closestDis = (q - p).norm();
    //         iter++;
    //     }
    //     return {closestParam, closestDis};
    // }

    [[nodiscard]] ClosestDescOnLine getClosestParam(const Vec3 &p) const override
    {
        ClosestDescOnLine closestDes{};
        for (int i = 0; i < _bugles.size(); ++i) {
            const Vec3 &a = _points[i];
            const Vec3 &b = _points[(i + 1) % _points.size()];
            if (isEqual(_bugles[i], 0)) {
                // 点到线段最近距离
                ClosestDescOnLine segPtDistRes = segPtDist(p, a, b);
                if (segPtDistRes.dis < closestDes.dis) {
                    closestDes   = segPtDistRes;
                    closestDes.t = i + segPtDistRes.t;
                }
                continue;
            }
            const CircularArc<Vec3> &arc           = circularArcs[i];
            const Vec3              &o             = arc.center;
            // p 到圆弧平面的投影
            const Vec3               projPt        = p - _refNormal.dot(p - a) * _refNormal;
            // projPt到圆的最近点
            const Vec3               clsPtOnCircle = o + arc.radius * (projPt - o).normalize();
            if ((clsPtOnCircle - a).dot(arc.inCircleDir) > 0) {
                // 在圆弧上
                real dis = (p - clsPtOnCircle).norm();
                if (dis < closestDes.dis) {
                    closestDes.dis = dis;
                    Vec3 oa        = a - o;
                    Vec3 oClsPt    = clsPtOnCircle - o;
                    real R2        = arc.radius * arc.radius;
                    real cosTheta  = (oa).dot(oClsPt) / R2;
                    real theta     = std::acos(cosTheta); // [0, pi]
                    if ((oa.cross(oClsPt)).dot(_refNormal) < 0) { theta = PI2 - theta; }
                    closestDes.t = i + theta / arc.theta;
                }
                continue;
            }
            real paDis = (p - a).norm();
            real pbDis = (p - b).norm();
            if (paDis < closestDes.dis) {
                closestDes.dis = paDis;
                closestDes.t   = i;
            } else {
                closestDes.dis = pbDis;
                closestDes.t   = i + 1;
            }
        }
        return closestDes;
    }

    void print() const
    {
        if (_closed)
            std::cout << "Closed Polyline: \n";
        else
            std::cout << "Open Polyline: \n";
        std::cout << "Points: {\n";
        for (int i = 0; i < _points.size(); ++i) {
            std::cout << _points[i].x() << ", " << _points[i].y() << ", " << _points[i].z() << ">";
            if (i != _points.size() - 1) std::cout << ", ";
        }
        std::cout << "}\n";
        std::cout << "的可变参数Bugles: {\n";
        for (int i = 0; i < _bugles.size(); ++i) {
            std::cout << _bugles[i];
            if (i != _bugles.size() - 1) std::cout << ", ";
        }
        std::cout << "}\n";
    }

    [[nodiscard]] bool isEndParam(real t) const override
    {
        return t < EPS_END_PARAM || t > static_cast<real>(_bugles.size()) - EPS_END_PARAM;
    }

    [[nodiscard]] AABB getAABB() const override { return aabb; }

private:
    void initCircularArcInfo(const Vec3 &a, const Vec3 &b, real bugle, const Vec3 &refNormal, CircularArc<Vec3> &res)
    {
        if (isEqual(bugle, 0)) {
            res.radius      = INFINITY;
            res.theta       = 0;
            res.h           = INFINITY;
            res.inCircleDir = refNormal.cross(b - a).normalize();
            res.u           = res.inCircleDir;
            res.v           = refNormal.cross(res.u);
            return;
        }

        Vec3 abHalf     = (b - a) * HALF;
        Vec3 abNorm     = abHalf.normalize();
        real theta      = std::atan(fabs(bugle)) * 4;
        res.inCircleDir = abNorm.cross(refNormal) * (bugle > 0 ? 1 : -1);

        if (fabs(bugle) == 1) {
            res.h = 0;
        } else {
            res.h = abHalf.norm() / std::tan(theta * HALF);
        }
        res.center = a + abHalf - res.inCircleDir * res.h;
        res.theta  = theta;
        res.radius = (res.center - a).norm();
        res.u      = (a - res.center).normalize();
        res.v      = refNormal.cross(res.u);
    }
};

class HelixLine : public ILine
{
public:
    HelixLine(const Vec3 &axisStart, const Vec3 &axisEnd, real r, real advancePerRound, const Vec3 &startDir)
        : _axisStart(axisStart),
          _frequency(PI2 / advancePerRound),
          _u(startDir),
          _r(r),
          _2pir_p(PI2 * _r / advancePerRound),
          _4pi2r_p2(_2pir_p * PI2 / advancePerRound),
          SEG_T(advancePerRound / SEG_PER_ROUND),
          SEG_T_HALF(SEG_T / 2)
    {
        auto star2nd       = axisEnd - _axisStart;
        _advanceLen        = star2nd.norm();
        _axisDir           = star2nd / _advanceLen;
        _v                 = _axisDir.cross(_u);
        real _4pi2r        = PI2 * PI2 * _r;
        // _k = _4pi2r / (advancePerRound * advancePerRound + _4pi2r * _r);
        _arcDeltaMaxFactor = _4pi2r / (advancePerRound * advancePerRound + _4pi2r * _r) * ONE_EIGHT;

        // init aabb
        aabb.extend(_axisStart);
        aabb.extend(axisEnd);
        aabb.extend(r);
    }

    [[nodiscard]] Vec3 eval(real t) const override
    {
        real theta = _frequency * t;
        return _axisStart + _axisDir * t + (_u * std::cos(theta) + _v * std::sin(theta)) * _r;
    };

    [[nodiscard]] Vec3 der1(real param) const override
    {
        real theta = _frequency * param;
        return _axisDir + _2pir_p * (_v * std::cos(theta) - _u * std::sin(theta));
    };

    [[nodiscard]] Vec3 der2(real param) const override
    {
        real theta = _frequency * param;
        return -_4pi2r_p2 * (_u * std::cos(theta) + _v * std::sin(theta));
    };

    [[nodiscard]] Vec3 tangent(real t) const override { return der1(t).normalize(); }

    [[nodiscard]] Vec3 normal(real t, const Vec3 &tan = -1.) const override
    {
        Vec3 der2Vec = this->der2(t);
        if (tan == -1.) {
            Vec3 realTan = tangent(t);
            return (der2Vec - der2Vec.dot(realTan) * realTan).normalize();
        }
        return (der2Vec - der2Vec.dot(tan) * tan).normalize();
    }

    [[nodiscard]] real startT() const override { return 0; }

    [[nodiscard]] real endT() const override { return 1; }

    [[nodiscard]] AABB getAABB() const override { return aabb; }

    ClosestDescOnLine getClosestParam(const Vec3 &p) const override
    {
        // discretization and traversal
        real                           startT   = 0;
        real                           endT     = SEG_T;
        auto                           segCount = static_cast<size_t>(std::ceil(_advanceLen / SEG_T));
        std::vector<ClosestDescOnLine> sampledSegs(segCount + 2); // 加上首尾
        std::vector<Vec3>              samplePoints(segCount + 2);

        ClosestDescOnLine closestSampleDes;
        for (size_t i = 0; i < segCount; ++i, startT = endT, endT += SEG_T) {
            real sampledT      = fmin(startT + SEG_T_HALF, _advanceLen);
            samplePoints[i]    = eval(sampledT);
            sampledSegs[i].dis = (samplePoints[i] - p).norm();
            sampledSegs[i].t   = sampledT;
            if (sampledSegs[i].dis < closestSampleDes.dis) { closestSampleDes = sampledSegs[i]; }
        }
        // 特别考虑两端，因为有更多的情形，查询点会离两端更近
        // 好处是可能降低后续迭代次数
        samplePoints[segCount]     = eval(0);
        samplePoints[segCount + 1] = eval(_advanceLen);
        sampledSegs[segCount]      = {0, (samplePoints[segCount] - p).norm()};
        sampledSegs[segCount + 1]  = {_advanceLen, (samplePoints[segCount + 1] - p).norm()};
        for (size_t i = segCount; i <= segCount + 1; ++i) {
            if (sampledSegs[i].dis < closestSampleDes.dis) { closestSampleDes = sampledSegs[i]; }
        }

        real deltaMaxCommon = (eval(0) - eval(SEG_T)).l2() * _arcDeltaMaxFactor;
        for (int i = 0; i < sampledSegs.size(); ++i) {
            if (i == segCount - 1) {
                // 最后一段
                deltaMaxCommon = (eval(_advanceLen) - eval(SEG_T * static_cast<real>(segCount - 1))).l2() * _arcDeltaMaxFactor;
            } else if (i == segCount || i == segCount + 1) {
                // 首尾点
                deltaMaxCommon = 0;
            }
            if (sampledSegs[i].dis - deltaMaxCommon < closestSampleDes.dis) {
                Vec3 q     = samplePoints[i];
                real t     = sampledSegs[i].t;
                Vec3 qDer1 = der1(t);
                Vec3 qDer2 = der2(t);
                real lDer1 = (q - p).dot(qDer1);
                int  iter  = 0;
                while (fabs(lDer1) > EPS_NEWTON_ITERATION) {
                    real dif  = lDer1 / (qDer1.dot(qDer1) + (q - p).dot(qDer2));
                    real tNew = t - dif;
                    if (tNew < 0 || tNew > _advanceLen) break;
                    t     = tNew;
                    q     = eval(t);
                    qDer1 = der1(t);
                    qDer2 = der2(t);
                    lDer1 = (q - p).dot(qDer1);
                    std::cout << "After iter " << iter << ", dL is " << lDer1 << std::endl;
                    iter++;
                }
                real dis = (q - p).norm();
                if (dis < closestSampleDes.dis) { closestSampleDes = {t, dis}; }
            }
        }
        return closestSampleDes;
    };

    [[nodiscard]] bool isEndParam(real t) const override { return t < EPS || t > _advanceLen - EPS; }

private:
    Vec3       _axisStart, _axisDir;
    real       _advanceLen, _frequency;
    real       _startTheta;
    Vec3       _u, _v; // 螺旋投影圆面上的两个正交单位向量，u X v = axisDir
    real       _r, _2pir_p, _4pi2r_p2, _arcDeltaMaxFactor;
    const int  SEG_PER_ROUND = 12;
    const real SEG_T, SEG_T_HALF;
    AABB       aabb;
};

class SingleLine : public ILine
{
public:
};

// 单段圆弧
class ArcLine : public PolyLine
{
public:
    ArcLine(const Vec3 &a, const Vec3 &b, real bugle, const Vec3 &refNormal)
        // : _a(a), _b(b), _bugle(bugle), _refNormal(refNormal.normalize()) {}
        : PolyLine(
            Pt3Array{
                {a, b}
    },
            std::vector<real>{bugle},
            refNormal,
            false)
    {
    }

    [[nodiscard]] const std::vector<real> &getBugles() const = delete;

    [[nodiscard]] const real &getBugle() const { return _bugles[0]; }

    // private:
    // Vec3 _a, _b, _refNormal;
    // real _bugle;
    // CircularArc<Vec3> _circularArc;
};

class PolynomialLine : public ILine
{
public:
    Vec3 eval(real t) const override { return {}; };

    Vec3 der1(real t) const override { return {}; };

    Vec3 der2(real t) const override { return {}; };

    Vec3 tangent(real t) const override { return {}; }

    Vec3 normal(real t, const Vec3 &tan = -1.) const override { return {}; }

    ClosestDescOnLine getClosestParam(const Vec3 &p) const override { return {}; };
};
