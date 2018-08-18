#ifndef __UTILITIES_HPP__
#define __UTILITIES_HPP__

#include "typedefs.hpp"

#include <cmath>
#include <limits>

namespace rbt {

const Real PI = 3.141592653589793;
const Real EPSILON = 0.00001;
const auto INF = std::numeric_limits<Real>::infinity();

Real toRadians(const Real& degrees);
Real toDegrees(const Real& radians);

Real inchesToMillimeters(const Real& inches);
Real millimetersToInches(const Real& millimeters);

bool approxZero(const Real& value);
bool approxEqual(const Real& a, const Real& b);

bool isInf(const Real& value);

int sign(const Real& a);

Real minusPiToPi(Real angle);

}

#endif /* __UTILITIES_HPP__ */
