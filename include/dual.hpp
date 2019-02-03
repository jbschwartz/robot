#ifndef __DUAL_HPP__
#define __DUAL_HPP__

#include "typedefs.hpp"
#include "quaternion.hpp"

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

  Dual<T>& operator*=(const Dual<T>& a);
  Dual<T>& operator+=(const Dual<T>& a);
  Dual<T>& operator-=(const Dual<T>& a);
};

template <>
Dual<Quaternion>::Dual();

template <typename T>
Dual<T> operator+(const Dual<T>& a, const Dual<T>& b);

template <typename T>
Dual<T> operator-(const Dual<T>& a, const Dual<T>& b);

template <typename T>
Dual<T> operator*(const Dual<T>& a, const Dual<T>& b);

template <typename T>
Dual<T> operator*(const Dual<T>& a, const Real& s);

template <typename T>
Dual<T> operator*(const Real& s, const Dual<T>& a);

template <typename T>
Dual<T> operator/(const Dual<T>& a, const Real& s);

template <typename T>
bool operator==(const Dual<T>& a, const Dual<T>& b);

template <typename T>
Dual<T> conjugate(const Dual<T>& a);

template <>
Dual<Quaternion> conjugate(const Dual<Quaternion>& a);

template <typename T>
Dual<T> norm(const Dual<Quaternion>& a);

Real norm(const Dual<Quaternion>& a);

#ifdef DEBUG
template <typename T>
std::ostream& operator<<(std::ostream& os, const Dual<T>& a);
#endif

#include "dual.tpp"

}

#endif /* __DUAL_HPP__ */
