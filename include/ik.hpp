#ifndef __IK_HPP__
#define __IK_HPP__

#include "typedefs.hpp"
#include "vector.hpp"
#include "utilities.hpp"
#include "joint.hpp"
#include "frame.hpp"

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

// Get angle sets for the first three joints. This determines the position of the wrist.
// This may return AngleSets that are not physically achievable (i.e. joints may be beyond their limits)
AngleSets positionSets(const Real& x, const Real& y, const Real& z, const std::vector<Joint>& joints);

// Construct AngleSets by combining all individual solutions to the waist, shoulder, and elbow subproblems.
AngleSets buildPositionSets(const Angles& waist, const Angles& shoulder, const Angles& elbow);

// Get joint angles from a given pose (inverse kinematics)
AngleSets angles(const Frame& pose, const std::vector<Joint>& joints);

// calculate RS coordinates for the given XYZ position.
Vector2 rsCoordinates(const Real& x, const Real& y, const Real& z, const Real& shoulderOffset, const Real& baseOffset);

// Check each solution against the joint limits and prune if any angles are outside their limit.
void removeIfBeyondLimits(AngleSets& sets, const std::vector<Vector2>& limits);

// Return true if the angle is within the given limits
bool withinLimits(const Real& angle, const Vector2& limits);

// Get wrist center point given the end-effector pose and offset along Z.
Vector3 wristCenterPoint(const Frame& pose, const Real& wristZOffset);

// Determine if the target point (x, y) is on the positive or negative side of the shoulder.
int shoulderDirection(const Real& x, const Real& y, const Joint& shoulder, const Real& waistAngle);

// Determine the zero position of the shoulder axis
Real shoulderZero(const std::vector<Joint>& joints);
}}

#endif /* __IK_HPP__ */
