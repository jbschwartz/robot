#include "../third_party/catch.hpp"
#include "../matchers/angles.hpp"
#include "../robots/abb_irb_120.hpp"
#include "../../include/ik.hpp"
#include "../../include/serial.hpp"
#include "../../include/utilities.hpp"
#include "../../include/typedefs.hpp"

using namespace rbt;
using namespace rbt::ik;

// TODO: Need a way to test solveArm specifically without having to make calls to other functionality (transformAngles..., removeIfBeyond...)
TEST_CASE("solveArm") {
  // Corresponds to all joint angles set to 45 degrees
  const auto expected = Angles({ toRadians(45), toRadians(45), toRadians(45), toRadians(45), toRadians(45), toRadians(45) });
  const auto expectedSize = 1;

  const auto pose = ABB_IRB_120.pose(expected);
  const Vector3 target = wristCenterPoint(pose, ABB_IRB_120.wristLength());

  const auto upperArmLength = ABB_IRB_120.upperArmLength();
  const auto foreArmLength = ABB_IRB_120.foreArmLength();
  const auto shoulderWristOffset = ABB_IRB_120.shoulderWristOffset();
  const auto shoulderZ = ABB_IRB_120.shoulderZ();

  auto result = solveArm(target, upperArmLength, foreArmLength, shoulderWristOffset, shoulderZ);

  transformAnglesToRobot(result, ABB_IRB_120);

  removeIfBeyondLimits(result, ABB_IRB_120.limits());

  SECTION("calculates joint angles for wristCenter position") {
    REQUIRE(result.size() == expectedSize);

    std::for_each(result.front().begin(), result.front().end(), [expected](auto angle) {
      REQUIRE(angle == Approx(expected.front()));
    });

  }
}
