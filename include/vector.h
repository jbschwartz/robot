#ifndef __VECTOR_H__
#define __VECTOR_H__

#include <array>

#include "typedefs.h"

template<typename T, std::size_t N>
class Vector {
  std::array<T, N> v;
public:
  Vector(const std::array<T, N>& v = {}) : v(v) {};

  const T& operator[](std::size_t index) const { return this->v[index]; }

  template <typename U, std::size_t M>
  friend bool operator==(const Vector<U, M>& a, const Vector<U, M>& b);
};

template <typename T, std::size_t N>
bool operator==(const Vector<T, N>& a, const Vector<T, N>& b) {
  return a.v == b.v;
}

typedef Vector<Real, 3> Vector3;

#endif /* __VECTOR_H__ */
