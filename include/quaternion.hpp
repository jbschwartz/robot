#ifndef __QUATERNION_H__
#define __QUATERNION_H__

#include "typedefs.hpp"
#include "vector.hpp"

#ifdef DEBUG
#include <iostream>
#endif

namespace rbt {

class Quaternion {
public:
  Real r, x, y, z;
  Quaternion() : r(1), x(0), y(0), z(0) {};
  Quaternion(Real r, Real x, Real y, Real z) : r(r), x(x), y(y), z(z) {};
  Quaternion(Real r, const Vector3& axis) : r(r), x(axis[0]), y(axis[1]), z(axis[2]) {};

  Quaternion operator-() const;

  Quaternion& operator*=(const Quaternion& a);
};

Quaternion operator+(const Quaternion& a, const Quaternion& b);
Quaternion operator-(const Quaternion& a, const Quaternion& b);
Quaternion operator*(const Quaternion& a, const Quaternion& b);
Quaternion operator*(const Quaternion& a, const Real& s);
Quaternion operator*(const Real& s, const Quaternion& a);
Quaternion operator/(const Quaternion& a, const Real& s);

bool operator==(const Quaternion& a, const Quaternion& b);

Quaternion conjugate(const Quaternion& a);
Quaternion normalize(const Quaternion& a);

#ifdef DEBUG
std::ostream& operator<<(std::ostream& os, const Quaternion& a);
#endif

}

#endif /* __QUATERNION_H__ */
