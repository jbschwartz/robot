#ifndef __IK_H_
#define __IK_H_

#include "typedefs.hpp"
#include "vector.hpp"
#include "utilities.hpp"

#include <limits>
#include <vector>

namespace rbt { namespace ik {

// A set of joint angles, one for each joint in a serial link manipulator
typedef std::vector<Real> AngleSet;
// A set of joint angle sets, one for each configuration
// (e.g. one for each solution to an inverse kinematics problem)
typedef std::vector<std::vector<Real>> AngleSets;

static_assert(std::numeric_limits<Real>::has_infinity, "Type Real does not have infinity value");
const auto SINGULAR = std::numeric_limits<Real>::infinity();

// Calculate the waist (joint 0) angles satisfying actuator limits for location (x,y)
AngleSets waistAngles(const Real&, const Real& y);
// Calculate the waist (joint 0) angles satisfying actuator limits for location (x,y) with a shoulder offset
AngleSets waistAngles(const Real&, const Real& y, const Real& offset);

}}

#endif /* __IK_H_ */
