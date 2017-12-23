#ifndef __UTILITIES_H_
#define __UTILITIES_H_

#include "typedefs.hpp"

#include <cmath>

namespace rbt {

const Real PI = 3.141592653589793;

Real toRadians(const Real& degrees);
Real toDegrees(const Real& radians);

Real inchesToMillimeters(const Real& inches);
Real millimetersToInches(const Real& millimeters);

}

#endif /* __UTILITIES_H_ */
