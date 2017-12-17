#include "utilities.h"

Real toRadians(const Real& degrees) {
  return degrees * PI / 180;
}

Real toDegrees(const Real& radians) {
  return radians * 180 / PI;
}
