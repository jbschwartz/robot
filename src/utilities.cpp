#include "utilities.hpp"

namespace rbt {

Real toRadians(const Real& degrees) {
  return degrees * PI / 180;
}

Real toDegrees(const Real& radians) {
  return radians * 180 / PI;
}

Real inchesToMillimeters(const Real& inches) {
  return inches * 25.4;
}

Real millimetersToInches(const Real& millimeters) {
  return millimeters / 25.4;
}

bool approxZero(const Real& value) {
  return std::abs(value) <= EPSILON;
}

bool approxEqual(const Real& a, const Real& b) {
  if(isInf(a) && isInf(b)) return true;
  if(isInf(a) && !isInf(b)) return false;
  if(!isInf(b) && isInf(b)) return false;

  return approxZero(a - b);
}

bool isInf(const Real& value) {
  return std::numeric_limits<Real>::has_infinity && (value == std::numeric_limits<Real>::infinity());
}

int sign(const Real& a) {
  const Real zero = Real(0);
  return (zero < a) - (a < zero);
}

// Clamp angle to the range (-PI, PI]
Real minusPiToPi(Real angle) {
  const auto revolution = 2 * PI;

  while(angle > PI) angle -= revolution;
  while(angle <= -PI) angle += revolution;

  return angle;
}


}
