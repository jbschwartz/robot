#include "../include/ik.hpp"
#include "../include/vector.hpp"
#include "../include/utilities.hpp"
#include "../include/transform.hpp"

#include <algorithm>
#include <iterator>

namespace rbt { namespace ik {

// Project the wrist center onto the XY plane, solve for the angle in the plane with a shoulder-wrist offset.
Angles solveWaist(const Real& x, const Real& y, const Real& wristOffset) {
  Real alpha = 0;

  if(!approxZero(wristOffset))
  {
    // Shoulder-wrist offsets create potential for unreachable locations (so we check)
    //    A point is unreachable if x^2 + y^2 < d^2,
    //    i.e. if the point is "inside" the (circle produced by the) offset wrist
    const auto delta = (x * x) + (y * y) - (wristOffset * wristOffset);
    if(delta < 0) return Angles(); // No solution

    alpha = std::atan2(wristOffset, std::sqrt(delta));
  } else {
    const bool shoulderIsSingular = approxZero(x) && approxZero(y);
    if(shoulderIsSingular) return Angles({ SINGULAR, SINGULAR }); // Infinite possible solutions
  }

  const auto phi = std::atan2(y, x);

  // Give solutions for both "left" and "right" shoulder configurations
  // Constrain solutions to (-PI, PI] as joint limits are typically symmetric about zero
  const auto first = minusPiToPi(phi - alpha);
  const auto second = minusPiToPi(phi + alpha + PI);

  return Angles({ first, second });
}

// This solves the first part of the inverse kinematics for a 2R manipulator.
// Uses the law of cosines
Real solveElbow(const Real& r, const Real& s, const Real& upperArmLength, const Real& foreArmLength) {
  // Law of cosines
  const auto cosTheta = ((r * r) + (s * s) - (upperArmLength * upperArmLength) - (foreArmLength * foreArmLength)) / (2 * upperArmLength * foreArmLength);

  // Use atan instead of acos as atan performs better for very small angle values
  // This will return nan if the target location is unreachable (i.e. cosTheta is outside the range [-1, 1])
  return std::atan2(std::sqrt(1 - cosTheta * cosTheta), cosTheta);
}

// Does not do any checking for points out of reach as there would be no valid elbow angle to pass in.
Angles solveShoulder(const Real& r, const Real& s, const Real& upperArmLength, const Real& foreArmLength, const Real& elbow) {
  // A 2R manipulator is singular if the target point coincides with the shoulder axis
  bool isSingular =
    approxZero(r) &&
    approxZero(s) &&
    approxEqual(upperArmLength, foreArmLength);

  if(isSingular) return Angles({ SINGULAR });

  auto angles = Angles();

  const auto phi = std::atan2(s, r);

  for(auto angle : std::vector<Real>({elbow, -elbow})) {
    const auto shoulder = phi - std::atan2(foreArmLength * std::sin(angle), upperArmLength + foreArmLength * std::cos(angle));
    angles.push_back(shoulder);
    // If angle is either 0 or Pi, then both elbow angles will generate the same shoulder angle.
    if(approxZero(angle) || approxEqual(angle, PI)) break;
  }

  return angles;
}

AngleSets solveArm(const Vector3& target, const Real& upperArmLength, const Real& foreArmLength, const Real& shoulderWristOffset, const Real& shoulderZOffset) {
  const Real& x = target[0];
  const Real& y = target[1];
  const Real& z = target[2];

  const auto waist = solveWaist(x, y, shoulderWristOffset);
  if(waist.empty()) return AngleSets();

  const auto rs = rsCoordinates(x, y, z, shoulderWristOffset, shoulderZOffset);
  const auto r = rs[0]; const auto s = rs[1];

  auto elbow = solveElbow(r, s, upperArmLength, foreArmLength);
  if(std::isnan(elbow)) return AngleSets();

  auto shoulder = solveShoulder(r, s, upperArmLength, foreArmLength, elbow);
  if(shoulder.empty()) return AngleSets();

  // The elbow can have an up and down configuration
  // Flip the shoulder handedness for the second waist solution
  // When the shoulder switches handedness, the elbow flips configuration
  return AngleSets({
    Angles({ waist[0], shoulder[0], elbow }),
    Angles({ waist[0], shoulder[1], -elbow }),
    Angles({ waist[1], PI - shoulder[0], -elbow }),
    Angles({ waist[1], PI - shoulder[1], elbow }),
  });
}

Vector3 wristCenterPoint(const Frame& pose, const Real& wristZOffset) {
  // TODO: Handle tools
  return pose.position() - pose.zAxis() * wristZOffset;
}

void transformAnglesToRobot(AngleSets& angleSets, const Serial& robot) {
  const auto waistZero = robot.waistZero();
  const auto shoulderDirection = robot.shoulderDirection();
  const auto shoulderZero = robot.shoulderZero();
  const auto elbowDirection = robot.elbowDirection();
  const auto elbowZero = robot.elbowZero();

  for(auto&& angles : angleSets) {
    angles[0] -= waistZero;
    angles[1] = shoulderDirection * angles[1] - shoulderZero;
    angles[2] = elbowDirection * (angles[2] + elbowZero);
  }
}

AngleSets angles(const Frame& pose, const Serial& robot) {
  // Transform end-effector tip frame to wrist center frame
  const auto target = wristCenterPoint(pose, robot.wristLength());

  auto solutions = solveArm(
    target,
    robot.upperArmLength(),
    robot.foreArmLength(),
    robot.shoulderWristOffset(),
    robot.shoulderZ()
  );

  transformAnglesToRobot(solutions, robot);
  
  removeIfBeyondLimits(solutions, robot.limits());

  if(solutions.empty()) return AngleSets();

  const auto joints = robot.joints();
  for(auto&& set : solutions) {
    auto t = Transform();
    t *= joints[0].transform(set[0]);
    t *= joints[1].transform(set[1]);
    t *= joints[2].transform(set[2]);
    t *= joints[3].transform();
    t *= joints[4].transform();
    t *= joints[5].transform();

    const auto wristCenterFrame = Frame(t.dual);

    const auto desiredWristPose = conjugate(wristCenterFrame.pose()) * pose.pose();

    const auto angles = euler<Intrinsic::ZYZ>(desiredWristPose);

    set.insert(set.end(), angles.begin(), angles.end());
  }

  return solutions;
}

/* Project the problem onto a 2R manipulator. That is: find the wrist center on the RS plane
 *             (Side)                                (Top)
 *               ^ Z          Target (x, y, z)        ^ Y
 *               |           /    => (r, s)           |
 *               ^ S (O)====X                         |
 *     Shoulder  |  // \                              |/---- r ----/
 *             \ |//    Elbow                 ---^--- (O)==(O)=====X (x, y)
 *    --^---    (O)-----------> R                |    ||       _ /
 *   offset    =====                           offset ||   _ /   \
 *      |      || ||                             |    || /        sqrt(x^2 + y^2)
 *   ---V--- ----|---------> X                ---V--- (O)-----------> X */
Vector2 rsCoordinates(const Real& x, const Real& y, const Real& z, const Real& shoulderWristOffset, const Real& baseZOffset) {
  const auto r = std::sqrt(x * x + y * y - shoulderWristOffset * shoulderWristOffset);
  const auto s = z - baseZOffset;

  return Vector2({ r, s });
}

bool withinLimits(const Real& angle, const Vector2& limits) {
  // Do not remove singular values as they represent _all_ values within limits
  if(angle == SINGULAR) return true;

  Real low = limits[0];
  Real high = limits[1];

  if(low > high) std::swap(low, high);

  return (angle >= low) && (angle <= high);
}

void removeIfBeyondLimits(AngleSets& sets, const std::vector<Vector2>& limits) {
  const auto last = std::remove_if(sets.begin(), sets.end(), [limits](const Angles& set) {
    auto limIter = limits.begin();
    for(auto& angle : set) {
      if (!withinLimits(angle, *limIter++)) return true;
    }
    return false;
  });

  sets.erase(last, sets.end());
}

}}
