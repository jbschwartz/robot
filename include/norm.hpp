#ifndef __NORM_H_
#define __NORM_H_

#include "typedefs.hpp"
#include "quaternion.hpp"
#include "dual.hpp"

#include <cmath>

namespace rbt {

Real norm(const Quaternion& a);
Real norm(const Dual<Quaternion>& a);

template <typename T>
Dual<T> norm(const Dual<T>& a) {
  return Dual<T>(a * conjugate(a));
}

}

#endif /* __NORM_H_ */
