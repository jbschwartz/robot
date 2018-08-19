#include "../third_party/catch.hpp"
#include "../matchers/angles.hpp"
#include "../robots/abb_irb_120.hpp"
#include "../../include/ik.hpp"
#include "../../include/serial.hpp"
#include "../../include/utilities.hpp"
#include "../../include/typedefs.hpp"

using namespace rbt;
using namespace rbt::ik;

TEST_CASE("solveArm") {
  // Corresponds to all joint angles set to 45 degrees
  const Vector3 target = Vector3({ 184.497, 184.497, 178.919 });

  const auto upperArmLength = ABB_IRB_120.upperArmLength();
  const auto foreArmLength = ABB_IRB_120.foreArmLength();
  const auto shoulderWristOffset = ABB_IRB_120.shoulderWristOffset();
  const auto shoulderZ = ABB_IRB_120.shoulderZ();

  const auto result = solveArm(target, upperArmLength, foreArmLength, shoulderWristOffset, shoulderZ);

  SECTION("calculates position solutions to inverse kinematics") {
    const auto expectedSize = 4;
    REQUIRE(result.size() == expectedSize);

    SECTION("needs tests") {
      REQUIRE(true == true);
    }
  }
}
