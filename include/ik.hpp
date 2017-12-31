#ifndef __IK_H_
#define __IK_H_

#include "typedefs.hpp"
#include "vector.hpp"

namespace rbt { namespace ik {

// Remove angles from a vector should the exceed actuator limits
// TODO: This could potentially be more generic
void removeIfOutsideLimits(Angles& angles, const Vector2& limits);

}}

#endif /* __IK_H_ */
