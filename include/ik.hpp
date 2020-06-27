#ifndef __IK_HPP__
#define __IK_HPP__

#include "typedefs.hpp"
#include "utilities.hpp"
#include "joint.hpp"
#include "frame.hpp"
#include "serial.hpp"
#include "spatial/vector.hpp"

#include <limits>

namespace rbt { namespace ik {

const auto SINGULAR = std::numeric_limits<Real>::infinity();

// Calculate the waist (joint 0) angles with a shoulder-wrist offset
Angles solveWaist(const Real& x, const Real& y, const Real& wristOffset = 0);

// Calculate one elbow (joint 2) angle. Callers will negate the returned value to get the other elbow configuration
Real solveElbow(const Real& r, const Real& s, const Real& upperArmLength, const Real& foreArmLength);

// Calculate the shoulder (joint 1) angles from elbow angle
Angles solveShoulder(const Real& r, const Real& s, const Real& upperArmLength, const Real& foreArmLength, const Real& elbow);

// Get solutions for the first three joints of a canonical arm
AngleSets solveArm(const Vector3& wristCenter, const Real& upperArmLength, const Real& foreArmLength, const Real& shoulderWristOffset, const Real& shoulderZOffset);

// Get joint angles from a given pose (inverse kinematics)
AngleSets angles(const Frame& pose, const Serial& joints);

// Take canonical angles and transform them to the Robot joint space
void transformAnglesToRobot(AngleSets& angleSets, const Serial& robot);

// Calculate RS coordinates for the given XYZ position.
Vector2 rsCoordinates(const Real& x, const Real& y, const Real& z, const Real& shoulderWristOffset, const Real& baseZOffset);

// Check each solution against the joint limits and prune if any angles are outside their limit
void removeIfBeyondLimits(AngleSets& sets, const std::vector<Vector2>& limits);

// Return true if the angle is within the given limits
bool withinLimits(const Real& angle, const Vector2& limits);

// Get wrist center point given the end-effector pose and offset along Z.
Vector3 wristCenterPoint(const Frame& pose, const Real& wristZOffset);

}}

#endif /* __IK_HPP__ */
