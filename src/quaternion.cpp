#include <cmath>

#include "../include/quaternion.h"

Real Quaternion::norm() const {
  return std::sqrt(
    this->r * this->r +
    this->x * this->x +
    this->y * this->y +
    this->z * this->z);
}

Quaternion Quaternion::operator-() const {
  return Quaternion(-this->r, -this->x, -this->y, -this->z);
}

Quaternion operator+(const Quaternion& a, const Quaternion& b) {
  return Quaternion(a.r + b.r, a.x + b.x, a.y + b.y, a.z + b.z);
}

Quaternion operator*(const Quaternion& a, const Real& s) {
  return Quaternion(a.r * s, a.x * s, a.y * s, a.z * s);
}

Quaternion operator*(const Real& s, const Quaternion& a) {
  return Quaternion(s * a.r, s * a.x, s * a.y, s * a.z);
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
  const auto norm = a.norm();
  return Quaternion(a.r / norm, a.x / norm, a.y / norm, a.z / norm);
}
