#include "../include/ik.hpp"
#include "../include/vector.hpp"
#include "../include/utilities.hpp"

#include <algorithm>

namespace rbt { namespace ik {

// Project the wrist center onto the XY plane, solve for the angle in the plane.
Angles waistAngles(const Real& x, const Real& y) {
  // No shoulder offset means possibility of shoulder singularities
  // Shoulder singularities occur when the center of the wrist intersects the waist axis
  bool shoulderIsSingular = (x == 0) && (y == 0);
  if(shoulderIsSingular) return Angles({SINGULAR}); // Infinite possible solutions

  const auto phi = std::atan2(y, x);

  // Give solutions for both "left" and "right" shoulder configurations
  return Angles({phi, phi + PI});
}

// Project the wrist center onto the XY plane, solve for the angle in the plane with a shoulder offset.
Angles waistAngles(const Real& x, const Real& y, const Real& offset) {
  if(offset == 0) return waistAngles(x, y);

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

bool withinLimits(const Real& angle, const Vector2& limits) {
  // Do not remove singular values as they represent _all_ values within limits
  if(angle == SINGULAR) return true;

  Real low = limits[0];
  Real high = limits[1];

  if(low > high) std::swap(low, high);

  return (angle >= low) && (angle <= high);
}

}}
