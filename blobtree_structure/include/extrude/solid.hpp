#pragma once

#include "common.hpp"
#include "real.hpp"
#include <array>
#include <cassert>
#include <cstddef>
#include <utility>
#include <vector>
#include "vec.hpp"
#include "line.hpp"

class ISolid
{
public:
    virtual ~ISolid()                 = default;
    ISolid()                          = default;
    ISolid(const ISolid &)            = default;
    ISolid &operator=(const ISolid &) = default;
    ISolid(ISolid &&)                 = default;
    ISolid &operator=(ISolid &&)      = default;

    // virtual real sdf(const Vec3 &p) const = 0;
    virtual real sdf(const Vec3 &p, Vec3 &closestPoint) const = 0;
};

inline Vec2 get2DRepOf3DPt(const Vec3 &pt3D, const Vec3 &u, const Vec3 &v, const Vec3 &localO)
{
    Vec3 OP = pt3D - localO;
    return {OP.dot(u), OP.dot(v)};
}

inline Vec2 get2DRepOf3DDir(const Vec3 &dir, const Vec3 &u, const Vec3 &v) { return Vec2{dir.dot(u), dir.dot(v)}.normalize(); }

inline Vec3 get3DRepOf2DPt(const Vec2 &pt2D, const Vec3 &u, const Vec3 &v, const Vec3 &localO)
{
    return localO + pt2D[0] * u + pt2D[1] * v;
}

/**
 *  calculate winding number of a point w.r.t. a segment ab
 */
// inline real unsignedWindingNumberSegment(const Vec3 &p, const Vec3 &a, const Vec3 &b,
//                                          const Vec3 &refNormal) {
//     Vec3 pa = a - p;
//     Vec3 pb = b - p;
//     return std::acos(std::clamp(pa.dot(pb) / (pa.norm() * pb.norm()), static_cast<real>(-1.),
//                                 static_cast<real>(1.)))
//            / (std::numbers::pi * 2);
// }

template <typename AxisLineType>
class IExtrudedSolidBase : public ISolid
{
protected:
    std::vector<PolyLine>                       _profiles; // TODO: may be replaced by const ref to profile
    AxisLineType                                _axis;
    real                                        _rScale;
    std::vector<Pt2Array>                       _localProfiles2D;
    std::vector<std::vector<CircularArc<Vec2>>> _localArcs2d;
    Vec3                                        _biNormalStartPt;
    Vec3                                        _axisStart, _axisStartTangent, _axisEnd, _axisEndTangent;

public:
    AABB aabb;

    IExtrudedSolidBase(std::vector<PolyLine> profiles, AxisLineType axis, real rScale)
        : ISolid(), _profiles(std::move(profiles)), _axis(std::move(axis)), _rScale(rScale)
    {
        assert(!_profiles.empty());
        for (const auto &_profile : _profiles) { assert(_profile.isClosed()); }
        // project profile at st point to 2D
        _axisStartTangent   = _axis.tangent(0);
        Vec3 normal         = _axis.normal(0);
        _biNormalStartPt    = _axisStartTangent.cross(normal);
        _axisStart          = _axis.eval(0);
        _axisEnd            = _axis.eval(_axis.endT());
        _axisEndTangent     = _axis.tangent(_axis.endT());
        size_t profileCount = _profiles.size();
        _localProfiles2D.resize(profileCount);
        _localArcs2d.resize(profileCount);
        for (int i = 0; i < _profiles.size(); ++i) {
            const auto &profile  = _profiles[i];
            size_t      segCount = profile.getPoints().size();
            _localProfiles2D[i].resize(segCount);
            _localArcs2d[i].resize(segCount);
            for (int j = 0; j < segCount; ++j) {
                // TODO:
                _localProfiles2D[i][j] = get2DRepOf3DPt(profile.getPoints()[j], normal, _biNormalStartPt, _axisStart);
                auto       &arc2d      = _localArcs2d[i][j];
                const auto &arc3d      = profile.getCircularArcs()[j];
                arc2d.center           = get2DRepOf3DPt(arc3d.center, normal, _biNormalStartPt, _axisStart);
                arc2d.inCircleDir      = get2DRepOf3DDir(arc3d.inCircleDir, normal, _biNormalStartPt);
                arc2d.radius           = arc3d.radius;
                arc2d.theta            = arc3d.theta;
                arc2d.h                = arc3d.h;
            }
        }
        AABB aabbOfProfile = _profiles[0].getAABB();
        aabb               = _axis.getAABB();
        aabb.move(aabbOfProfile.center() - _axisStart);
        aabb.expand(aabbOfProfile.halfSize());
        aabb.expand();
    }

    real sdf(const Vec3 &p, Vec3 &closestPoint) const override
    {
        ClosestDescOnLine closestDescToAxis = _axis.getClosestParam(p);
        // TNB coordinate system
        auto              t                 = closestDescToAxis.t;
        Vec3              q                 = _axis.eval(t); // closest point on axis
        Vec3              qp                = p - q;
        auto              TBN               = getTBN(p, q, closestDescToAxis.t);
        const Vec3       &tangent           = TBN[0];
        const Vec3       &normal            = TBN[1];
        const Vec3       &biNormal          = TBN[2];
        if (_axis.isEndParam(t) && fabs(qp.dot(tangent)) > EPS) {
            // cases away from two sides
            // project p to the plane passing through Q and perpendicular to T
            real               pqDotT            = -qp.dot(tangent);
            Vec3               projP             = p + tangent * pqDotT;
            Vec2               p2D               = get2DRepOf3DPt(projP, normal, biNormal, q);
            PtBoundaryRelation ptProfileRelation = pmcProfile2d(p2D);
            real               projectedDis      = 0;
            if (ptProfileRelation == Outside) {
                auto closestDesOnProfile = disProfile2D(p2D);
                projectedDis             = closestDesOnProfile.dis;
                closestPoint             = _profiles[closestDesOnProfile.i].eval(closestDesOnProfile.t);
            } else {
                closestPoint = projP;
            }
            // p must be positive (outside)
            return sqrt(projectedDis * projectedDis + pqDotT * pqDotT);
        }

        Vec2               p2D               = get2DRepOf3DPt(p, normal, biNormal, q);
        PtBoundaryRelation ptProfileRelation = pmcProfile2d(p2D);
        if (ptProfileRelation == OnBoundary) {
            closestPoint = p;
            return 0;
        }
        auto closestDescToProfile = disProfile2D(p2D);
        // consider whether either of two sides is closer
        if (ptProfileRelation == Inside) {
            real pqDotT = (_axisStart - p).dot(_axisStartTangent);
            real disTmp = sqrt(pqDotT * pqDotT);
            if (disTmp < closestDescToProfile.dis) {
                closestPoint = p - tangent * pqDotT;
                return -disTmp; // return here may be fault when is closer to the other side, but this does not exit for thick
                                // extrusion solid
            }
            pqDotT = (_axisEnd - p).dot(_axisEndTangent);
            disTmp = sqrt(pqDotT * pqDotT);
            if (disTmp < closestDescToProfile.dis) {
                closestPoint = p + tangent * pqDotT;
                return -disTmp;
            }
        }
        closestPoint = _profiles[closestDescToProfile.i].eval(closestDescToProfile.t);
        return closestDescToProfile.dis * static_cast<int>(ptProfileRelation);
    }

private:
    virtual std::array<Vec3, 3> getTBN(const Vec3 &p, const Vec3 &q, real t) const = 0;

    [[nodiscard]] inline ClosestDescOnProfile disProfile2D(const Vec2 &p2D) const
    {
        ClosestDescOnProfile closestDescToProfile{};
        for (int i = 0; i < _localArcs2d.size(); ++i) {
            ClosestDescOnLine closestDescTemp = disLoop2D(p2D, _localProfiles2D[i], _localArcs2d[i]);
            if (closestDescTemp.dis < closestDescToProfile.dis) {
                closestDescToProfile.dis = closestDescTemp.dis;
                closestDescToProfile.t   = closestDescTemp.t;
                closestDescToProfile.i   = i;
            }
        }
        return closestDescToProfile;
    }

    inline PtBoundaryRelation pmcProfile2d(const Vec2 &p2D) const
    {
        // PMC
        for (int i = 0; i < _localArcs2d.size(); ++i) {
            PtBoundaryRelation relationTmp = pmcLoop2d(p2D, _localProfiles2D[i], _localArcs2d[i]);
            if (relationTmp == OnBoundary) {
                return OnBoundary; // TODO: 判断OnBoundary的过程可以加一点容差
            }
            if ((relationTmp == Outside && i == 0) || (relationTmp == Inside && i != 0)) { return Outside; }
        }
        return Inside;
    }

    /** 2D Loop的内外部判定
     * in + in = out
     * in + out = in
     * out + in = in
     * out + out = out
     */
    static PtBoundaryRelation pmcLoop2d(const Vec2 &p2D, const Pt2Array &loop2D, const std::vector<CircularArc<Vec2>> &arcs2d)
    {
        size_t segCount             = arcs2d.size();
        // 先判断是否在outline上
        // 顺便判断点-扇的位置关系
        bool   inFan                = false;
        int    onLinesegButHasBugle = -1;
        for (int i = 0; i < segCount; ++i) {
            const Vec2 &a = loop2D[i];
            const Vec2 &b = loop2D[(i + 1) % segCount];
            if (arcs2d[i].theta <= EPS) {
                // straight line segment
                if (isPointOnSegment(p2D, a, b)) { return OnBoundary; }
                continue;
            }
            if (isPointOnSegment(p2D, a, b)) {
                onLinesegButHasBugle = i;
                break;
            }
            const auto &arc = arcs2d[i];
            real        po  = (p2D - arc.center).norm();
            if ((p2D - a).dot(arc.inCircleDir) > 0) {
                if (po == arc.radius) { return OnBoundary; }
                if (po < arc.radius) {
                    inFan = true;
                    break;
                }
            } else {
                if (po <= arc.radius) {
                    inFan = true;
                    break;
                }
            }
        }

        // 判断点-直线多边形的关系
        const auto ptInPolygon = [&](const Vec2 &p) {
            int           intersectionCount = 0;
            // int onSegIdx = -1;
            constexpr int numRays           = 3; // 射线数量
            int           majorityIn        = 0; // 在多边形内的射线计数
            int           majorityOut       = 0; // 在多边形外的射线计数
            for (int rayIdx = 0; rayIdx < numRays; ++rayIdx) {
                double angle = (PI2 * rayIdx) / numRays;
                Vec2   rayDir(cos(angle), sin(angle));
                int    crossings = 0;

                for (int i = 0; i < segCount; ++i) {
                    const Vec2 &a = loop2D[i];
                    const Vec2 &b = loop2D[(i + 1) % segCount];
                    assert(!isPointOnSegment(p, a, b));
                    // if (isPointOnSegment(p2D, a, b))
                    // {
                    //     onSegIdx = i;
                    //     break;
                    // }
                    // 使用向量方法计算射线和边的交点
                    double dx1         = b[0] - a[0];
                    double dy1         = b[1] - a[1];
                    double dx2         = rayDir[0];
                    double dy2         = rayDir[1];
                    double determinant = dx1 * dy2 - dy1 * dx2;

                    // 如果determinant为0，则射线和边平行，不计算交点
                    if (isEqual(determinant, 0)) continue;

                    double t1 = ((p[0] - a[0]) * dy2 - (p[1] - a[1]) * dx2) / determinant;
                    double t2 = ((p[0] - a[0]) * dy1 - (p[1] - a[1]) * dx1) / determinant;

                    // 检查交点是否在边上（0 <= t1 <= 1）且射线上（t2 >= 0）
                    if (t1 >= 0 && t1 <= 1 && t2 >= 0) { crossings++; }
                }
                if (crossings % 2 == 0) {
                    majorityOut++;
                } else {
                    majorityIn++;
                }
            }
            return majorityIn > majorityOut;
        };

        if (onLinesegButHasBugle != -1) {
            // 需要特殊考虑的情况
            // 从p2D向inCircle方向前进一小步
            Vec2 samplePt = p2D + arcs2d[onLinesegButHasBugle].inCircleDir * EPS;
            return ptInPolygon(samplePt) ? Outside : Inside; // 反过来的
        }
        return ptInPolygon(p2D) ^ inFan ? Inside : Outside;
    }

    static ClosestDescOnLine disLoop2D(const Vec2 &p2D, const Pt2Array &loop2D, const std::vector<CircularArc<Vec2>> &arcs2d)
    {
        size_t segCount = arcs2d.size();
        assert(loop2D.size() == segCount);
        ClosestDescOnLine res{};
        for (int i = 0; i < segCount; ++i) {
            auto disDesc = distance2Arc2D(p2D, loop2D[i], loop2D[(i + 1) % segCount], arcs2d[i]);
            if (res.dis > disDesc.dis) {
                res.dis = disDesc.dis;
                res.t   = i + disDesc.t;
            }
        }
        return res;
    }

    static ClosestDescOnLine distance2Arc2D(const Vec2 &p2D, const Vec2 &a, const Vec2 &b, const CircularArc<Vec2> &arc)
    {
        if (isEqual(arc.theta, 0)) { return segPtDist(p2D, a, b); }
        const Vec2 &center = arc.center;
        Vec2        op     = p2D - center;
        Vec2        q      = center + arc.radius * op.normalize(); // closest pt on circle
        // 判断q是否在弧上
        if ((q - a).dot(arc.inCircleDir) > 0) {
            // 计算参数
            Vec2 oq       = q - center;
            Vec2 oa       = a - center;
            real R2       = arc.radius * arc.radius;
            real cosTheta = (oa).dot(oq) / R2;
            real sinTheta = (oa).cross(oq) / R2;
            return {atan2(sinTheta, cosTheta) / arc.theta, (p2D - q).norm()};
        }

        real paDis = (a - p2D).norm();
        real pbDis = (b - p2D).norm();
        if (paDis < pbDis) return {0, paDis};
        return {1, pbDis};
    }

    static bool isPointOnSegment(const Vec2 &p, const Vec2 &a, const Vec2 &b)
    {
        // check collinearity
        double crossProduct = (p[1] - a[1]) * (b[0] - a[0]) - (p[0] - a[0]) * (b[1] - a[1]);
        if (!isEqual(crossProduct, 0)) {
            return false; // Not collinear
        }

        // Check if point is within segment bounds
        return (p[0] >= std::min(a[0], b[0]) && p[0] <= std::max(a[0], b[0]) && p[1] >= std::min(a[1], b[1])
                && p[1] <= std::max(a[1], b[1]));
    }

    //     bool isOn2DPolyline(const Vec2 &p2D, const Pt2Array &profile2D,
    //                     const std::vector<CircularArc<Vec2>> &arcs2d) {
    //     size_t segCount = arcs2d.size();
    //     for (int i = 0; i < segCount; ++i) {
    //         const Vec2 &a = profile2D[i];
    //         const Vec2 &b = profile2D[(i + 1) % segCount];
    //         if (arcs2d[i].h <= EPS) {
    //             //line segment
    //             if (isPointOnSegment(p2D, a, b)) {
    //                 return true;
    //             }
    //             continue;
    //         }
    //     }
    //     // TODO: 没写完，但是暂时用不到
    //     return true;
    // }
};

class ExtrudedSolidPolyline : public IExtrudedSolidBase<PolyLine>
{
public:
    ExtrudedSolidPolyline(std::vector<PolyLine> profiles, PolyLine axis, real rScale = 1.0)
        : IExtrudedSolidBase(std::move(profiles), std::move(axis), rScale)
    {
        assert(_biNormalStartPt.isParallel(_axis.getRefNormal()));
    }

    std::array<Vec3, 3> getTBN(const Vec3 &p, const Vec3 &q, real t) const override
    {
        if (!_axis.isEndParam(t) && std::abs(t - std::round(t)) < EPS) {
            // 衔接点处（注意排除断点处）
            // p到圆弧平面的投影
            Vec3 projPt = p - _biNormalStartPt.dot(p - q) * _biNormalStartPt;
            Vec3 normal = (q - projPt).normalize();
            if (normal.dot(_axis.normal(t)) < 0) { normal = -normal; }
            return {normal.cross(_biNormalStartPt), normal, _biNormalStartPt};
        }
        Vec3 tangent = _axis.tangent(t);
        return {tangent, _axis.normal(t, tangent), _biNormalStartPt};
    }

    // real wnCircularArc(
    //     const Vec3 &p, const Vec3 &a, const Vec3 &b, const Vec3 &plgNormal, const PolyLine::CircularArc &arc, int dir)
    // {
    //     Vec3 pa = a - p;
    //     Vec3 pb = b - p;
    //     real wn =
    //         std::acos(std::clamp(pa.dot(pb) / (pa.norm() * pb.norm()), static_cast<real>(-1.), static_cast<real>(1.))) /
    //         (std::numbers::pi * 2);
    //     auto inOutCircle = arc.inCircleCheck(p);
    //     if (inOutCircle == PtBoundaryRelation::Outside || pa.cross(pb).dot(plgNormal) < 0)
    //     {
    //         // outside
    //         // pa.cross(pb).dot(plgNormal) 不会 == 0
    //         return -wn * dir;
    //     }
    //     if (inOutCircle == PtBoundaryRelation::Inside)
    //     {
    //         return wn * dir;
    //     }

    //     return 0;
    // }

    // Vec2 eval2DProfile(real param)
    // {
    //     int seg = static_cast<int>(param);
    //     real tOnSeg = param - seg;
    //     const auto &arc = circularArcs[seg];
    //     real phi = tOnSeg * arc.theta;
    //     return arc.center + arc.radius * (arc.u * std::cos(phi) + arc.v * std::sin(phi));
    // }
};

class ExtrudedSolidArcLine : public ExtrudedSolidPolyline
{
public:
    ExtrudedSolidArcLine(std::vector<PolyLine> profiles, ArcLine axis, real rScale = 1.0)
        : ExtrudedSolidPolyline(std::move(profiles), std::move(axis), rScale)
    {
    }
};

class ExtrudedSolidHelixLine : public IExtrudedSolidBase<HelixLine>
{
public:
    ExtrudedSolidHelixLine(std::vector<PolyLine> profiles, HelixLine axis, real rScale = 1.0)
        : IExtrudedSolidBase(std::move(profiles), std::move(axis), rScale)
    {
    }

    std::array<Vec3, 3> getTBN([[maybe_unused]] const Vec3 &a, [[maybe_unused]] const Vec3 &b, real t) const override
    {
        Vec3 tangent = _axis.tangent(t);
        Vec3 normal  = _axis.normal(t);
        return {tangent, normal, tangent.cross(normal)};
    }
};
