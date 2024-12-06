#pragma once
#include "real.hpp"
#include <limits>
#include <cmath>

enum PtBoundaryRelation { Inside = -1, OnBoundary, Outside = 1 };

const real PI                   = 3.141592653589793238462643383279502884;
const real PI2                  = 2 * PI;
const real EPS                  = std::numeric_limits<real>::epsilon() * 1e2;
const real EPS_END_PARAM        = std::numeric_limits<real>::epsilon() * 1e3;
const real EPS_NEWTON_ITERATION = std::numeric_limits<real>::epsilon() * 1e6;
const real EPS_AABB_EXTENSION   = std::numeric_limits<real>::epsilon() * 1e6;

const real HALF       = 0.5;
const real ONE_FOURTH = 0.25;
const real ONE_EIGHT  = 0.125;

inline bool isEqual(real a, real b) { return fabs(a - b) < EPS; }