#include <cmath>

#include "../include/quaternion.hpp"

#ifdef DEBUG
#include <iostream>
#endif

namespace rbt {

Quaternion Quaternion::operator-() const {
  return Quaternion(-this->r, -this->x, -this->y, -this->z);
}

Quaternion& Quaternion::operator*=(const Quaternion& a) {
  const auto r = this->r * a.r - this->x * a.x - this->y * a.y - this->z * a.z;
  const auto x = this->r * a.x + this->x * a.r + this->y * a.z - this->z * a.y;
  const auto y = this->r * a.y - this->x * a.z + this->y * a.r + this->z * a.x;
  const auto z = this->r * a.z + this->x * a.y - this->y * a.x + this->z * a.r;

  this->r = r; this->x = x; this->y = y; this->z = z;

  return *this;
}

Quaternion& Quaternion::operator+=(const Quaternion& a) {
  this->r += a.r; this->x += a.x; this->y += a.y; this->z += a.z;

  return *this;
}

Quaternion& Quaternion::operator-=(const Quaternion& a) {
  this->r -= a.r; this->x -= a.x; this->y -= a.y; this->z -= a.z;

  return *this;
}

Quaternion operator+(const Quaternion& a, const Quaternion& b) {
  return Quaternion(a.r + b.r, a.x + b.x, a.y + b.y, a.z + b.z);
}

Quaternion operator-(const Quaternion& a, const Quaternion& b) {
  return Quaternion(a.r - b.r, a.x - b.x, a.y - b.y, a.z - b.z);
}

Quaternion operator*(const Quaternion& a, const Real& s) {
  return Quaternion(a.r * s, a.x * s, a.y * s, a.z * s);
}

Quaternion operator*(const Real& s, const Quaternion& a) {
  return Quaternion(s * a.r, s * a.x, s * a.y, s * a.z);
}

Quaternion operator/(const Quaternion& a, const Real& s) {
  const Real reciprocal = 1. / s;
  return Quaternion(a.r * reciprocal, a.x * reciprocal, a.y * reciprocal, a.z * reciprocal);
}

Quaternion operator*(const Quaternion& a, const Quaternion& b) {
  const auto r = a.r * b.r - a.x * b.x - a.y * b.y - a.z * b.z;
  const auto x = a.r * b.x + a.x * b.r + a.y * b.z - a.z * b.y;
  const auto y = a.r * b.y - a.x * b.z + a.y * b.r + a.z * b.x;
  const auto z = a.r * b.z + a.x * b.y - a.y * b.x + a.z * b.r;

  return Quaternion(r, x, y, z);
}

bool operator==(const Quaternion& a, const Quaternion& b) {
  return a.r == b.r && a.x == b.x && a.y == b.y && a.z == b.z;
}

Quaternion conjugate(const Quaternion& a) {
  return Quaternion(a.r, -a.x, -a.y, -a.z);
}

Quaternion normalize(const Quaternion& a) {
  const auto n = norm(a);
  return Quaternion(a.r / n, a.x / n, a.y / n, a.z / n);
}

Real norm(const Quaternion& a) {
  return std::sqrt(
    a.r * a.r +
    a.x * a.x +
    a.y * a.y +
    a.z * a.z
  );
}

#ifdef DEBUG
std::ostream& operator<<(std::ostream& os, const Quaternion& a) {
  os << "(" << a.r << " + " << a.x << "i + " << a.y << "j + " << a.z << "k)";
  return os;
}
#endif

}
