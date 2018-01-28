#include "third_party/catch.hpp"
#include "matchers/vector.hpp"
#include "../include/joint.hpp"
#include "../include/transform.hpp"
#include "../include/vector.hpp"

using rbt::Joint;
using rbt::Real;
using rbt::Transform;
using rbt::Vector2;
using rbt::Vector3;
using rbt::toRadians;

TEST_CASE("Joint") {
  SECTION("Forward Kinematics") {
    auto j1 = Joint(toRadians( -90),    0, toRadians(   0),  290, Vector2({ -165, 165 }));
    auto j2 = Joint(toRadians(   0),  270, toRadians( -90),    0, Vector2({ -110, 110 }));
    auto j3 = Joint(toRadians( -90),   70, toRadians(   0),    0, Vector2({  -90, 70  }));
    auto j4 = Joint(toRadians(  90),    0, toRadians(   0),  302, Vector2({ -160, 160 }));
    auto j5 = Joint(toRadians( -90),    0, toRadians(   0),    0, Vector2({ -120, 120 }));
    auto j6 = Joint(toRadians(   0),    0, toRadians( 180),   72, Vector2({ -400, 400 }));

    auto j1t = j1.transform();
    auto j2t = j2.transform();
    auto j3t = j3.transform();
    auto j4t = j4.transform();
    auto j5t = j5.transform();
    auto j6t = j6.transform();

    auto origin = Vector3({0, 0, 0});

    auto result = j1t(j2t(j3t(j4t(j5t(j6t(origin))))));
    auto expected = Vector3({374, 0, 630});

    CHECK_THAT(result, ComponentsEqual(expected));
  }
}
