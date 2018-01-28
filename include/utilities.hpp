#ifndef __UTILITIES_H_
#define __UTILITIES_H_

#include "typedefs.hpp"

#include <cmath>
#include <limits>

namespace rbt {

const Real PI = 3.141592653589793;
const Real COS_MAX = 1;
const Real EPSILON = 0.00001;

static_assert(std::numeric_limits<Real>::has_infinity, "Type Real does not have infinity value");
const auto INF = std::numeric_limits<Real>::infinity();

Real toRadians(const Real& degrees);
Real toDegrees(const Real& radians);

Real inchesToMillimeters(const Real& inches);
Real millimetersToInches(const Real& millimeters);

bool approxZero(const Real& value);
bool approxEqual(const Real& a, const Real& b);

int sign(const Real& a);

}

#endif /* __UTILITIES_H_ */
