#include "../include/ik.hpp"
#include "../include/vector.hpp"
#include "../include/utilities.hpp"
#include "../include/transform.hpp"

#include <algorithm>
#include <iterator>

namespace rbt { namespace ik {

// Project the wrist center onto the XY plane, solve for the angle in the plane.
Angles waistAngles(const Real& x, const Real& y) {
  // No shoulder offset means possibility of shoulder singularities
  // Shoulder singularities occur when the center of the wrist intersects the waist axis
  bool shoulderIsSingular = approxZero(x) && approxZero(y);
  if(shoulderIsSingular) return Angles({SINGULAR}); // Infinite possible solutions

  const auto phi = std::atan2(y, x);

  // Give solutions for both "left" and "right" shoulder configurations
  // To abide by joint angle limits, make sure to provide the second solution in the opposite direction
  return (phi > 0) ? Angles({phi, phi - PI}) : Angles({phi, phi + PI});
}

// Project the wrist center onto the XY plane, solve for the angle in the plane with a shoulder offset.
Angles waistAngles(const Real& x, const Real& y, const Real& offset) {
  if(approxZero(offset)) return waistAngles(x, y);

  // Shoulder offsets create potential for unreachable locations (so we check)
  // A point is unreachable is x^2 + y^2 < d^2,
  //   i.e. if the point is "inside" the (circle produced by the) offset shoulder
  const auto delta = (x * x) + (y * y) - (offset * offset);

  if(delta < 0) return Angles(); // No solution

  const auto phi = std::atan2(y, x);
  const auto alpha = std::atan2(offset, std::sqrt(delta));

  // Give solutions for both "left" and "right" shoulder configurations
  return Angles({phi - alpha, phi + alpha + PI});
}

// This solves the first part of the inverse kinematics for a 2R manipulator.
// Uses the law of cosines
Angles elbowAngles(const Real& r, const Real& s, const Real& l1, const Real& l2) {
  // Law of cosines
  const auto cosTheta = ((r * r) + (s * s) - (l1 * l1) - (l2 * l2)) / (2 * l1 * l2);

  // There is no solution if cosTheta is outside the range of (-1, 1) inclusive
  if(!withinLimits(cosTheta, Vector2({ -COS_MAX, COS_MAX }))) return Angles();

  // Arm is on the edge of the external boundary; upper and lower arms are colinear and not overlapping
  if(approxEqual(cosTheta, COS_MAX)) return Angles({ 0 });
  // Arm is on the edge of the internal boundary; upper and lower arms are colinear and overlapping
  if(approxEqual(cosTheta, -COS_MAX)) return Angles({ toRadians(180), toRadians(-180) });

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
Angles shoulderAngles(const Real& r, const Real& s, const Real& l1, const Real& l2, const Angles& elbow) {
  // Must check this here as .front() and .back() are called below. Stay away UB!
  if(elbow.empty()) return Angles();

  // A 2R manipulator is singular if the target point coincides with the shoulder axis
  bool isSingular =
    approxZero(r) &&
    approxZero(s) &&
    approxEqual(l1, l2);

  if(isSingular) return Angles({ SINGULAR });

  const auto phi = std::atan2(s, r);

  // Lambda for calculating a shoulder angle from one elbow angle
  const auto shoulder = [phi, l1, l2](auto& elbow) {
    return phi - std::atan2(l2 * std::sin(elbow), l1 + l2 * std::cos(elbow));
  };

  const auto theta1 = shoulder(elbow.front());

  if(elbow.size() == 2) {
    const auto theta2 = shoulder(elbow.back());
    // Check if both elbow angles produce the same result
    if(!approxEqual(theta1, theta2))
      return Angles({ theta1, theta2 });
  }

  return Angles({ theta1 });
}

int shoulderDirection(const Real& x, const Real& y, const Joint& shoulder, const Real& waistAngle) {
  const auto shoulderFrame = Frame(shoulder.transform(waistAngle).dual);
  const auto zAxis = shoulderFrame.zAxis();
  // Take the cross product between the Z Axis and the point (x, y)
  const auto cross = zAxis[0] * y - zAxis[1] * x;
  // The sign determines which side the point is on: left or right
  return sign(cross);
}

Real shoulderZero(const std::vector<Joint>& joints) {
  const auto shoulderFrame = Frame(joints[0].transform().dual);
  const auto elbowFrame = Frame((joints[0].transform() * joints[1].transform()).dual);

  const auto shoulderLink = elbowFrame.position() - shoulderFrame.position();
  return angleBetween(shoulderLink, Vector3({ 1, 0, 0 }));
}

AngleSets positionSets(const Real& x, const Real& y, const Real& z, const std::vector<Joint>& joints) {
  const auto shoulderOffset = joints[1].offset() + joints[2].offset();
  const auto baseOffset = joints[0].offset();
  const auto l1 = joints[1].length();
  const auto l2 = std::sqrt((joints[2].length() * joints[2].length()) + (joints[3].offset() * joints[3].offset()));

  const auto a1 = joints[2].length();
  const auto a2 = joints[3].offset();

  const auto waist = waistAngles(x, y, shoulderOffset);
  if(waist.empty()) return AngleSets();

  const auto rs = rsCoordinates(x, y, z, shoulderOffset, baseOffset);
  const auto r = rs[0]; const auto s = rs[1];

  auto elbow = elbowAngles(r, s, l1, l2);
  auto shoulder = shoulderAngles(r, s, l1, l2, elbow);

  // Transform the shoulder angles based on where shoulder zero point is
  const auto direction = shoulderDirection(x, y, joints[0], waist.front());
  const auto shoulderAngleOffset = shoulderZero(joints);

  std::transform(shoulder.begin(), shoulder.end(), shoulder.begin(), [direction, shoulderAngleOffset](auto angle) {
    return (direction < 0) ? shoulderAngleOffset - angle : angle - shoulderAngleOffset;
  });

  // Transform the elbow angles to actuator angles.
  std::transform(elbow.begin(), elbow.end(), elbow.begin(), [a2, a1, direction](auto angle) {
    return (direction < 0) ? -angle - std::atan(a2 / a1) : std::atan(a2 / a1) + angle;
  });

  // We could optimize by checking immediately following ____Angles() calls but this suffices for now
  // There are no solutions if any of these are empty; return an empty set
  if(elbow.empty() || shoulder.empty()) return AngleSets();

  return buildPositionSets(waist, shoulder, elbow);
}

AngleSets buildPositionSets(const Angles& waist, const Angles& shoulder, const Angles& elbow) {
  auto sets = AngleSets();

  // When the shoulder switches handedness, it's necessary to flip shoulder and elbow angles
  // We assume that the first waist angle is always in the "correct" quadrant
  // TODO: write a function to check if waist angle is in the proper quadrant instead
  bool flip = false;
  for(auto w = waist.begin(); w != waist.end(); ++w) {
    for(auto s = shoulder.begin(), e = elbow.begin(); s != shoulder.end(); ++s, ++e) {
      if(flip) {
        sets.emplace_back(AngleSet({ *w, PI - *s, -*e }));
      } else {
        sets.emplace_back(AngleSet({ *w, *s, *e }));
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
Vector2 rsCoordinates(const Real& x, const Real& y, const Real& z, const Real& shoulderOffset, const Real& baseOffset) {
  const auto r = std::sqrt(x * x + y * y - shoulderOffset * shoulderOffset);
  const auto s = z - baseOffset;

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
