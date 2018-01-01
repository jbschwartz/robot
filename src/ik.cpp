#include "../include/ik.hpp"
#include "../include/vector.hpp"
#include "../include/utilities.hpp"

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
  return Angles({phi, phi + PI});
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

AngleSets buildPositionSets(const Angles& waist, const Angles& shoulder, const Angles& elbow) {
  auto sets = AngleSets();

  // When the shoulder switches handedness, it's necessary to flip shoulder and elbow angles
  // We assume that the first waist angle is always in the "correct" quadrant
  // TODO: write a function to check if waist angle is in the proper quadrant instead
  bool flip = false;
  for(auto w = waist.begin(); w != waist.end(); ++w) {
    for(auto s = shoulder.begin(), e = elbow.begin(); s != shoulder.end(); ++s, ++e) {
      if(flip) {
        const auto PI = toRadians(180);
        sets.emplace_back(AngleSet({ *w, PI - *s, PI - *e }));
      } else {
        sets.emplace_back(AngleSet({ *w, *s, *e }));
      }
    }
    flip = !flip;
  }

  return sets;
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
  const auto r = std::sqrt(x * x + y + y - shoulderOffset * shoulderOffset);
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
