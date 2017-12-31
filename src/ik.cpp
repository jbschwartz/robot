#include "../include/ik.hpp"
#include "../include/vector.hpp"
#include "../include/utilities.hpp"

#include <algorithm>

namespace rbt { namespace ik {

// Project the wrist center onto the XY plane, solve for the angle in the plane.
Angles waistAngles(const Real& x, const Real& y, const Vector2& limits) {
  // No shoulder offset means possibility of shoulder singularities
  // Shoulder singularities occur when the center of the wrist intersects the waist axis
  bool shoulderIsSingular = (x == 0) && (y == 0);
  if(shoulderIsSingular) return Angles({SINGULAR}); // Infinite possible solutions

  const auto phi = std::atan2(y, x);

  // Give solutions for both "left" and "right" shoulder configurations
  auto angles = Angles({phi, phi + PI});

  // Make sure that angles are within actuator limits
  removeIfOutsideLimits(angles, limits);

  return angles;
}

// Project the wrist center onto the XY plane, solve for the angle in the plane with a shoulder offset.
Angles waistAngles(const Real& x, const Real& y, const Real& offset, const Vector2& limits) {
  // Shoulder offsets create potential for unreachable locations (so we check)
  // A point is unreachable is x^2 + y^2 < d^2,
  //   i.e. if the point is "inside" the (circle produced by the) offset shoulder
  const auto delta = (x * x) + (y * y) - (offset * offset);

  if(delta < 0) return Angles(); // No solution

  const auto phi = std::atan2(y, x);
  const auto alpha = std::atan2(offset, std::sqrt(delta));

  // Give solutions for both "left" and "right" shoulder configurations
  auto angles = Angles({phi - alpha, phi + alpha + PI});

  // Make sure that angles are within actuator limits
  removeIfOutsideLimits(angles, limits);

  return angles;
}

void removeIfOutsideLimits(Angles& angles, const Vector2& limits) {
  Real low = limits[0];
  Real high = limits[1];

  if(low > high) std::swap(low, high);

  // Optimization: don't bother looping through items if there are infnite limits
  if(low == -INF && high == INF) return;

  const auto last = std::remove_if(angles.begin(), angles.end(), [low, high](const Real& theta) {
    // Do not remove singular values as they represent all values _inside_ range
    if(theta == SINGULAR) return false;

    return (theta < low) || (theta > high);
  });

  angles.erase(last, angles.end());
}

}}
