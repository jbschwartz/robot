#ifndef __TYPEDEFS_HPP__
#define __TYPEDEFS_HPP__

#include <array>
#include <limits>
#include <vector>

namespace rbt {

typedef float Real;
static_assert(std::numeric_limits<Real>::has_infinity, "Type Real must have an infinity value");

typedef std::array<Real, 3> EulerAngles;

typedef Real Angle;
typedef std::vector<Angle> Angles;

// A set of joint angles, one for each configuration
// (e.g. one for each solution to an inverse kinematics problem)
typedef std::vector<std::vector<Angle>> AngleSets;


}

#endif /* __TYPEDEFS_HPP__ */
