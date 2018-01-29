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
bool operator==(const Vector<T, N>& a, const Vector<T, N>& b);

template <typename T, typename U, std::size_t N>
Vector<T, N> operator*(const U& s, const Vector<T, N>& a);

template <typename T, std::size_t N>
T operator*(const Vector<T, N>& a, const Vector<T, N>& b);

template <typename T, typename U, std::size_t N>
Vector<T, N> operator*(const Vector<T, N>& a, const U& s);

template <typename T, typename U, std::size_t N>
Vector<T, N> operator/(const Vector<T, N>& a, const U& s);

template <typename T, std::size_t N>
Vector<T, N> operator-(const Vector<T, N> a, const Vector<T, N> b);

template <typename T, std::size_t N>
T lengthSq(const Vector<T, N>& a);

template <typename T, std::size_t N>
T length(const Vector<T, N>& a);

template <typename T, std::size_t N>
Vector<T, N> unit(const Vector<T,N>& a);

template <typename T, std::size_t N>
T angleBetween(const Vector<T, N>& a, const Vector<T, N>& b);

#ifdef DEBUG
template <typename T, std::size_t N>
std::ostream& operator<<(std::ostream& os, const Vector<T, N>& a);
#endif

typedef Vector<Real, 2> Vector2;
typedef Vector<Real, 3> Vector3;

#include "vector.tpp"

}

#endif /* __VECTOR_H__ */
