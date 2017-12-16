#ifndef __QUATERNION_H__
#define __QUATERNION_H__

typedef float Real;

class Quaternion {
public:
  Real r, x, y, z;
  Quaternion() : r(1), x(0), y(0), z(0) {};
  Quaternion(Real r, Real x, Real y, Real z) : r(r), x(x), y(y), z(z) {};

  Real norm() const;
};

Quaternion operator+(const Quaternion& a, const Quaternion& b);
Quaternion operator*(const Quaternion& a, const Quaternion& b);

bool operator==(const Quaternion& a, const Quaternion& b);

Quaternion conjugate(const Quaternion& a);
Quaternion normalize(const Quaternion& a);

#endif /* __QUATERNION_H__ */
