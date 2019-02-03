#include "../third_party/catch.hpp"
#include "../matchers/angles.hpp"
#include "../robots/abb_irb_120.hpp"
#include "../../include/ik.hpp"
#include "../../include/serial.hpp"
#include "../../include/utilities.hpp"
#include "../../include/typedefs.hpp"

using namespace rbt;
using namespace rbt::ik;

TEST_CASE("angles") {
  const auto expected = Angles({ toRadians(45), toRadians(45), toRadians(45), toRadians(45), toRadians(45), toRadians(45) });

  const auto frame = ABB_IRB_120.pose(expected);
  const auto result = angles(frame, ABB_IRB_120);

  SECTION("calculates position solutions to inverse kinematics") {
    CHECK_THAT(result.front(), ComponentsEqual(expected));
  }
}