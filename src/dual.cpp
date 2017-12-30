#include "../include/dual.hpp"

namespace rbt {

template <>
Dual<Quaternion> conjugate(const Dual<Quaternion>& a) {
  return Dual<Quaternion>(conjugate(a.r), -conjugate(a.d));
}

template <>
Dual<Quaternion>::Dual() : r(Quaternion()), d(Quaternion(0, 0, 0, 0)) {};

Real norm(const Dual<Quaternion>& a) {
  return norm(a.r);
}

}
