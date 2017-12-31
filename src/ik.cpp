#include "../include/ik.hpp"
#include "../include/vector.hpp"

namespace rbt { namespace ik {

void removeIfOutsideLimits(Angles& angles, const Vector2& limits) {
  const Real low = limits[0];
  const Real high = limits[1];

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
