#ifndef __JOINT_H_
#define __JOINT_H_

#include "typedefs.hpp"
#include "utilities.hpp"
#include "vector.hpp"

namespace rbt {

class Transform;

class Joint {
  Real alpha, a, theta, d;
  Vector2 lims;
public:
  Joint(const Real& alpha, const Real& a, const Real& theta, const Real& d, const Vector2& limits = Vector2({-360.f, 360.f})) : alpha(alpha), a(a), theta(theta), d(d), lims(limits) {};
  Transform transform(const Real& theta = 0.0) const;

  Real offset() const { return this->d; };
  Real length() const { return this->a; };
  Vector2 limits() const { return this->lims; };
};

}

#endif /* __JOINT_H_ */
