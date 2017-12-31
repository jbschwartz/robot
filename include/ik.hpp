#ifndef __IK_H_
#define __IK_H_

#include "typedefs.hpp"
#include "vector.hpp"
#include "utilities.hpp"

#include <limits>
#include <vector>

namespace rbt { namespace ik {

typedef std::vector<Real> Angles;

static_assert(std::numeric_limits<Real>::has_infinity, "Type Real does not have infinity value");
const auto SINGULAR = std::numeric_limits<Real>::infinity();

const auto NO_LIMITS = Vector2({-INF, INF});

// Calculate the waist (joint 0) angles satisfying actuator limits for location (x,y)
Angles waistAngles(const Real&, const Real& y, const Vector2& limits = NO_LIMITS);
// Calculate the waist (joint 0) angles satisfying actuator limits for location (x,y) with a shoulder offset
Angles waistAngles(const Real&, const Real& y, const Real& offset, const Vector2& limits = NO_LIMITS);

// Remove angles from a vector should the exceed actuator limits
// TODO: This could potentially be more generic
void removeIfOutsideLimits(Angles& angles, const Vector2& limits);

}}

#endif /* __IK_H_ */
