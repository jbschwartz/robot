#include "third_party/catch.hpp"
#include "matchers/vector.hpp"
#include "robots/abb_irb_120.hpp"
#include "../include/joint.hpp"
#include "../include/transform.hpp"
#include "../include/vector.hpp"

using rbt::Joint;
using rbt::Real;
using rbt::Transform;
using rbt::Vector2;
using rbt::Vector3;
using rbt::toRadians;
using rbt::ABB_IRB_120;

TEST_CASE("Joint") {
  SECTION("Forward Kinematics") {
    const auto joints = ABB_IRB_120.joints();

    auto j1t = joints[0].transform();
    auto j2t = joints[1].transform();
    auto j3t = joints[2].transform();
    auto j4t = joints[3].transform();
    auto j5t = joints[4].transform();
    auto j6t = joints[5].transform();

    auto origin = Vector3({0, 0, 0});

    auto result = j1t(j2t(j3t(j4t(j5t(j6t(origin))))));
    auto expected = Vector3({374, 0, 630});

    CHECK_THAT(result, ComponentsEqual(expected));
  }
}
