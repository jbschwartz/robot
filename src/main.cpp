#include "../include/quaternion.hpp"
#include "../include/dual.hpp"
#include "../include/serial.hpp"
#include "../include/joint.hpp"
#include "../include/vector.hpp"
#include "../include/transform.hpp"

#include <iostream>

using rbt::Transform;
using rbt::Vector3;
using rbt::Vector2;
using rbt::Serial;
using rbt::Joint;

int main() {
  // ABB IRB120
  auto s = Serial({
    Joint( -90,    0,    0,  290, Vector2({ -165, 165 })),
    Joint(   0,  270,  -90,    0, Vector2({ -110, 110 })),
    Joint( -90,   70,    0,    0, Vector2({  -90, 70  })),
    Joint(  90,    0,    0,  302, Vector2({ -160, 160 })),
    Joint( -90,    0,    0,    0, Vector2({ -120, 120 })),
    Joint(   0,    0,  180,   72, Vector2({ -400, 400 }))
  });

  auto result = s.pose({45, 45, 45, 45, 45, 45});

  std::cout << result.position() << std::endl;
  std::cout << result.orientation() << std::endl;
}
