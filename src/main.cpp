#include "serial.hpp"
#include "joint.hpp"
#include "spatial/dual.hpp"
#include "spatial/quaternion.hpp"
#include "spatial/transform.hpp"
#include "spatial/vector.hpp"

#include <iostream>

using rbt::Transform;
using rbt::Vector3;
using rbt::Vector2;
using rbt::Serial;
using rbt::Joint;
using rbt::toRadians;

int main() {
  // ABB IRB120
  const auto ABB_IRB_120 = Serial({
    Joint(toRadians( -90),    0, toRadians(   0),  290, Vector2({ toRadians(-165), toRadians(165) })),
    Joint(toRadians(   0),  270, toRadians( -90),    0, Vector2({ toRadians(-110), toRadians(110) })),
    Joint(toRadians( -90),   70, toRadians(   0),    0, Vector2({ toRadians(-110), toRadians(70)  })),
    Joint(toRadians(  90),    0, toRadians(   0),  302, Vector2({ toRadians(-160), toRadians(160) })),
    Joint(toRadians( -90),    0, toRadians(   0),    0, Vector2({ toRadians(-120), toRadians(120) })),
    Joint(toRadians(   0),    0, toRadians( 180),   72, Vector2({ toRadians(-400), toRadians(400) }))
  });

  const auto angle = toRadians(45);
  auto result = ABB_IRB_120.pose({angle, angle, angle, angle, angle, angle});

  std::cout << result.position() << std::endl;
  std::cout << result.orientation() << std::endl;
}
