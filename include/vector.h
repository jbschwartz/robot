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
};

typedef Vector<Real, 3> Vector3;

#endif /* __VECTOR_H__ */
