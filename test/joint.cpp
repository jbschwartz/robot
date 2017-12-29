#include "third_party/catch.hpp"
#include "matchers/vector.hpp"
#include "../include/joint.hpp"
#include "../include/transform.hpp"
#include "../include/vector.hpp"

using rbt::Joint;
using rbt::Real;
using rbt::Transform;
using rbt::Vector3;

TEST_CASE("Joint") {
  SECTION("Forward Kinematics") {
    auto j1 = Joint(-90,	0, 		0, 		290);
    auto j2 = Joint(0,   	270,	-90, 	0);
    auto j3 = Joint(90,		-70, 	180,	0);
    auto j4 = Joint(-90, 	0,  	0, 		302);
    auto j5 = Joint(90, 	0, 		0, 		0);
    auto j6 = Joint(0,   	0, 		0,   	72);

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
