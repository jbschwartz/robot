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
    if(shoulderIsSingular) return Angles({ SINGULAR }); // Infinite possible solutions
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
Angles solveElbow(const Real& r, const Real& s, const Real& l1, const Real& l2) {
  // Law of cosines
  const auto cosTheta = ((r * r) + (s * s) - (l1 * l1) - (l2 * l2)) / (2 * l1 * l2);

  // There is no solution if cosTheta is outside the range of (-1, 1) inclusive
  if(!withinLimits(cosTheta, Vector2({ -COS_MAX, COS_MAX }))) return Angles();

  // Arm is on the edge of the external boundary; upper and lower arms are colinear and not overlapping
  if(approxEqual(cosTheta, COS_MAX)) return Angles({ 0 });
  // Arm is on the edge of the internal boundary; upper and lower arms are colinear and overlapping
  if(approxEqual(cosTheta, -COS_MAX)) return Angles({ PI, -PI });

  // Use atan instead of acos as atan performs better for very small angle values
  // Also the atan formulation naturally produces the elbow up and down solutions
  // i.e. +/- sqrt(1 - cosTheta^2)
  const auto y = std::sqrt(1 - cosTheta * cosTheta);

  return Angles({
    std::atan2(y, cosTheta),
    std::atan2(-y, cosTheta)
  });
}

// Does not do any checking for points out of reach as there would be no valid elbow angle to pass in.
Angles solveShoulder(const Real& r, const Real& s, const Real& l1, const Real& l2, const Angles& elbow) {
  // Must check this here as .front() and .back() are called below. Stay away UB!
  if(elbow.empty()) return Angles();

  // A 2R manipulator is singular if the target point coincides with the shoulder axis
  bool isSingular =
    approxZero(r) &&
    approxZero(s) &&
    approxEqual(l1, l2);

  if(isSingular) return Angles({ SINGULAR });

  auto angles = Angles();

  const auto phi = std::atan2(s, r);

  for(auto angle : elbow) {
    const auto shoulder = phi - std::atan2(l2 * std::sin(angle), l1 + l2 * std::cos(angle));
    angles.push_back(shoulder);

    // If angle is either 0 or Pi, then both elbow angles will generate the same shoulder angle.
    if(approxZero(angle) || approxEqual(angle, PI)) break;
  }

  return angles;
}

AngleSets positionSets(const Real& x, const Real& y, const Real& z, const std::vector<Joint>& joints) {
  const auto shoulderWristOffset = joints[1].offset() + joints[2].offset();

  const auto waist = solveWaist(x, y, shoulderWristOffset);
  if(waist.empty()) return AngleSets();

  const auto baseOffset = joints[0].offset();
  const auto rs = rsCoordinates(x, y, z, shoulderWristOffset, baseOffset);
  const auto r = rs[0]; const auto s = rs[1];

  const auto l1 = joints[1].length();
  const auto l2 = std::sqrt((joints[2].length() * joints[2].length()) + (joints[3].offset() * joints[3].offset()));

  auto elbow = solveElbow(r, s, l1, l2);
  if(elbow.empty()) return AngleSets();

  auto shoulder = solveShoulder(r, s, l1, l2, elbow);
  if(shoulder.empty()) return AngleSets();

  return buildPositionSets(waist, shoulder, elbow, joints);
}

AngleSets buildPositionSets(const Angles& waist, const Angles& shoulder, const Angles& elbow, const std::vector<Joint>& joints) {
  const auto shoulderDir = sign(joints[0].twist());
  const auto shoulderZeroOffset = joints[1].angle();

  const auto a1 = joints[2].length();
  const auto a2 = joints[3].offset();

  const auto elbowDir = (joints[1].twist() == PI) ? -shoulderDir : shoulderDir;
  const auto elbowZeroOffset = std::atan(a2 / a1);


  auto sets = AngleSets();

  // When the shoulder switches handedness, it's necessary to flip shoulder and elbow angles
  // We assume that the first waist angle is always in the "correct" quadrant
  // TODO: write a function to check if waist angle is in the proper quadrant instead
  bool flip = false;
  for(auto w : waist) {
    w -= joints[0].angle();
    for(auto s = shoulder.begin(), e = elbow.begin(); s != shoulder.end(); ++s, ++e) {
      const auto sNew = shoulderDir * *s - shoulderZeroOffset;
      const auto eNew = elbowDir * (*e + elbowZeroOffset);

      if(flip) {
        sets.emplace_back(AngleSet({ w, PI - sNew, -eNew }));
      } else {
        sets.emplace_back(AngleSet({ w, sNew, eNew }));
      }
    }
    flip = !flip;
  }

  return sets;
}

Vector3 wristCenterPoint(const Frame& pose, const Real& wristZOffset) {
  // TODO: Handle tools
  return pose.position() - pose.zAxis() * wristZOffset;
}

AngleSets angles(const Frame& pose, const std::vector<Joint>& joints) {
  // Transform end-effector tip frame to wrist center frame
  const auto wristCenter = wristCenterPoint(pose, joints[5].offset());

  auto angleSets = positionSets(wristCenter[0], wristCenter[1], wristCenter[2], joints);
  const auto limits = std::vector<Vector2>({ joints[0].limits(), joints[1].limits(), joints[2].limits() });

  removeIfBeyondLimits(angleSets, limits);

  // There are no solutions, return an empty set
  if(angleSets.empty()) return AngleSets();

  for(auto&& set : angleSets) {
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

  return angleSets;
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
  const auto last = std::remove_if(sets.begin(), sets.end(), [limits](const AngleSet& set) {
    auto limIter = limits.begin();
    for(auto& angle : set) {
      if (!withinLimits(angle, *limIter++)) return true;
    }
    return false;
  });

  sets.erase(last, sets.end());
}

}}
