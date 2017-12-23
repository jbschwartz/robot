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

}
