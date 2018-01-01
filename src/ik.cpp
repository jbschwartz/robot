#include "../include/ik.hpp"
#include "../include/vector.hpp"
#include "../include/utilities.hpp"

#include <algorithm>

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
  if(!withinLimits(cosTheta, Vector2({ -1, 1 }))) return Angles();

  // Arm is on the edge of the external boundary; upper and lower arms are colinear and not overlapping
  if(approxEqual(cosTheta, 1)) return Angles({ 0 });
  // Arm is on the edge of the internal boundary; upper and lower arms are colinear and overlapping
  if(approxEqual(cosTheta, -1)) return Angles({ toRadians(180), toRadians(-180) });

  // Use atan instead of acos as atan performs better for very small angle values
  // Also the atan formulation naturally produces the elbow up and down solutions
  // i.e. +/- sqrt(1 - cosTheta^2)
  const auto y = std::sqrt(1 - cosTheta * cosTheta);

  return Angles({
    std::atan2(y, cosTheta),
    std::atan2(-y, cosTheta)
  });
}

bool withinLimits(const Real& angle, const Vector2& limits) {
  // Do not remove singular values as they represent _all_ values within limits
  if(angle == SINGULAR) return true;

  Real low = limits[0];
  Real high = limits[1];

  if(low > high) std::swap(low, high);

  return (angle >= low) && (angle <= high);
}

}}
