#ifndef __DUAL_H__
#define __DUAL_H__

#include "typedefs.h"
#include "quaternion.h"

#ifdef DEBUG
#include <iostream>
#endif

namespace rbt {

template <typename T>
class Dual {
public:
  T r, d;
  Dual() : r(T()), d(T()) {}
  Dual(const T& r, const T& d) : r(r), d(d) {};

  Dual<T> norm() const {
    return Dual<T>(*this * conjugate(*this));
  }
};

template <typename T>
Dual<T> operator+(const Dual<T>& a, const Dual<T>& b) {
  return Dual<T>(a.r + b.r, a.d + b.d);
}

template <typename T>
Dual<T> operator*(const Dual<T>& a, const Dual<T>& b) {
  return Dual<T>(a.r * b.r, a.r * b.d + a.d * b.r);
}

template <typename T>
Dual<T> operator*(const Dual<T>& a, const Real& s) {
  return Dual<T>(s * a.r, s * a.d);
}

template <typename T>
Dual<T> operator*(const Real& s, const Dual<T>& a) {
  return Dual<T>(s * a.r, s * a.d);
}

template <typename T>
bool operator==(const Dual<T>& a, const Dual<T>& b) {
  return a.r == b.r && a.d == b.d;
}

template <typename T>
Dual<T> conjugate(const Dual<T>& a) {
  return Dual<T>(a.r, -a.d);
}

template <>
Dual<Quaternion> conjugate(const Dual<Quaternion>& a);

#ifdef DEBUG
template <typename T>
std::ostream& operator<<(std::ostream& os, const Dual<T>& a) {
  os << a.r << " + " << a.d << "\u03B5";
  return os;
};
#endif

}

#endif /* __DUAL_H__ */
