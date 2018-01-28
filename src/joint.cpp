#include "../include/joint.hpp"
#include "../include/transform.hpp"
#include "../include/quaternion.hpp"
#include "../include/dual.hpp"

#include <cmath>

namespace rbt {

// Create transformation from Denavit-Hartenberg parameters
// Transform = Translate_z(d) * Rotate_z(theta) * Translate_x(a) * Rotate_x(alpha);
Transform Joint::transform(const Real& theta) const {
  auto transform = Transform();
  auto sum = this->theta + theta;

  if(this->d != 0)      transform *= Transform(Vector3({0, 0, this->d}));
  if(sum != 0)          transform *= Transform(Vector3({0, 0, 1}), sum);
  if(this->a != 0)      transform *= Transform(Vector3({this->a, 0, 0}));
  if(this->alpha != 0)  transform *= Transform(Vector3({1, 0, 0}), this->alpha);

  return transform;
}

}
