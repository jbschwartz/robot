#ifndef __TRANSFORM_H__
#define __TRANSFORM_H__

#include "dual.hpp"
#include "vector.hpp"
#include "utilities.hpp"

#include <cmath>
#ifdef DEBUG
#include <iostream>
#endif

namespace rbt {

class Transform {
  Dual<Quaternion> dual;
public:
  Transform(const Dual<Quaternion>& dual) : dual(dual) {};
  Transform(const Vector3& axis, Real angle, const Vector3& translation = Vector3());
  Transform(const Vector3& translation) : Transform(Vector3(), 0, translation) {};

  #ifdef DEBUG
  friend std::ostream& operator<<(std::ostream& os, const Transform& t);
  #endif

  friend Transform operator*(const Transform& a, const Transform& b);

  Vector3 operator()(const Vector3& p) const;
};

#ifdef DEBUG
std::ostream& operator<<(std::ostream& os, const Transform& t);
#endif

Transform operator*(const Transform& a, const Transform& b);

}

#endif /* __TRANSFORM_H__ */
