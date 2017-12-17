#include "../include/dual.h"

namespace rbt {

template <>
Dual<Quaternion> conjugate(const Dual<Quaternion>& a) {
  return Dual<Quaternion>(conjugate(a.r), -conjugate(a.d));
}

}
