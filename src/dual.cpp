#include "../include/dual.h"

template <>
Dual<Quaternion> conjugate(const Dual<Quaternion>& a) {
  return Dual<Quaternion>(conjugate(a.r), -conjugate(a.d));
}
