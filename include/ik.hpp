#ifndef __IK_H_
#define __IK_H_

#include "typedefs.hpp"
#include "vector.hpp"
#include "utilities.hpp"

#include <limits>
#include <vector>

namespace rbt { namespace ik {

typedef Real Angle;
typedef std::vector<Angle> Angles;
// A set of joint angles, one for each joint in a serial link manipulator
typedef std::vector<Angle> AngleSet;
// A set of joint angle sets, one for each configuration
// (e.g. one for each solution to an inverse kinematics problem)
typedef std::vector<std::vector<Angle>> AngleSets;

static_assert(std::numeric_limits<Real>::has_infinity, "Type Real does not have infinity value");
const auto SINGULAR = std::numeric_limits<Real>::infinity();

// Calculate the waist (joint 0) angles
Angles waistAngles(const Real& x, const Real& y);
// Calculate the waist (joint 0) angles with a shoulder offset
Angles waistAngles(const Real& x, const Real& y, const Real& offset);

// Calculate the elbow (joint 2) angles
Angles elbowAngles(const Real& r, const Real& s, const Real& l1, const Real& l2);

// Calculate the shoulder (joint 1) angles from elbow angles
Angles shoulderAngles(const Real& r, const Real& s, const Real& l1, const Real& l2, const Angles& elbow);

// Return true if the angle is within the given limits
bool withinLimits(const Real& angle, const Vector2& limits);
}}

#endif /* __IK_H_ */
