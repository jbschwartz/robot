#ifndef __VECTOR_H__
#define __VECTOR_H__

#include <array>
#include <cmath>

#include "typedefs.hpp"
#include "utilities.hpp"

namespace rbt {

template<typename T, std::size_t N>
class Vector {
  std::array<T, N> v;
public:
  Vector(const std::array<T, N>& v = {}) : v(v) {};

  const T& operator[](std::size_t index) const { return this->v[index]; }
  T& operator[](std::size_t index) { return this->v[index]; }

  template <typename U, std::size_t M>
  friend bool operator==(const Vector<U, M>& a, const Vector<U, M>& b);
};

template <typename T, std::size_t N>
bool operator==(const Vector<T, N>& a, const Vector<T, N>& b) {
  return a.v == b.v;
}

template <typename T, typename U, std::size_t N>
Vector<T, N> operator*(const U& s, const Vector<T, N>& a) {
  auto v = Vector<T,N>();

  for(std::size_t i = 0; i < N; i++) {
    v[i] = s * a[i];
  }

  return v;
}

template <typename T, std::size_t N>
T operator*(const Vector<T, N>& a, const Vector<T, N>& b) {
  auto dot = T();

  for(std::size_t i = 0; i < N; i++) {
    dot += a[i] * b[i];
  }

  return dot;
}

template <typename T, typename U, std::size_t N>
Vector<T, N> operator*(const Vector<T, N>& a, const U& s) {
  return s * a;
}

template <typename T, typename U, std::size_t N>
Vector<T, N> operator/(const Vector<T, N>& a, const U& s) {
  const auto q = 1 / s;
  return q * a;
}

template <typename T, std::size_t N>
Vector<T, N> operator-(const Vector<T, N> a, const Vector<T, N> b) {
  auto v = Vector<T, N>();

  for(std::size_t i = 0; i < N; ++i) {
    v[i] = a[i] - b[i];
  }

  return v;
}

template <typename T, std::size_t N>
T lengthSq(const Vector<T, N>& a) {
  auto lengthSq = T();

  for(std::size_t i = 0; i < N; ++i) {
    lengthSq += a[i] * a[i];
  }

  return lengthSq;
}

template <typename T, std::size_t N>
T length(const Vector<T, N>& a) {
  return std::sqrt(rbt::lengthSq(a));
}

template <typename T, std::size_t N>
Vector<T, N> unit(const Vector<T,N>& a) {
  return a / length(a);
}

template <typename T, std::size_t N>
T angleBetween(const Vector<T, N>& a, const Vector<T, N>& b) {
  const auto dot = a * b;
  if(approxZero(dot)) return toRadians(90);

  return std::acos(dot / (length(a) * length(b)));
}

#ifdef DEBUG
template <typename T, std::size_t N>
std::ostream& operator<<(std::ostream& os, const Vector<T, N>& a) {
  for(std::size_t i = 0; i < N; ++i) {
    if(i != N) {
      os << a[i] << " ";
    } else {
      os << a[i];
    }
  }
  return os;
}
#endif

typedef Vector<Real, 2> Vector2;
typedef Vector<Real, 3> Vector3;

}

#endif /* __VECTOR_H__ */
