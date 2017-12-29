#ifndef __JOINT_H_
#define __JOINT_H_

#include "typedefs.hpp"
#include "utilities.hpp"

namespace rbt {

class Transform;

class Joint {
  Real alpha, a, theta, d;
public:
  Joint(const Real& alpha, const Real& a, const Real& theta, const Real& d) : alpha(alpha), a(a), theta(theta), d(d) {};
  Transform transform(const Real& theta = 0.0);
};

}

#endif /* __JOINT_H_ */
