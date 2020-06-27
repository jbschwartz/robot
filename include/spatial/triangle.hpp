#ifndef __TRIANGLE_HPP__
#define __TRIANGLE_HPP__

#include "spatial/vector.hpp"

#include <array>

namespace rbt
{

class Triangle {
  std::array<Vector3, 3> vertices;
public:
  Triangle(std::array<Vector3, 3> vertices) : vertices(std::move(vertices)) {};
};

}

#endif /* __TRIANGLE_HPP__ */
