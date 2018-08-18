#include "../third_party/catch.hpp"
#include "../matchers/angles.hpp"
#include "../../include/ik.hpp"
#include "../../include/utilities.hpp"
#include "../../include/typedefs.hpp"

using namespace rbt;
using namespace rbt::ik;

TEST_CASE("solveArm") {
  const Vector3 target = Vector3({ 302, 0, 630 });
  const auto result = solveArm(target, ABB_IRB_120.joints());

  SECTION("calculates position solutions to inverse kinematics") {
    const auto expectedSize = 4;
    REQUIRE(result.size() == expectedSize);

    SECTION("needs tests") {
      REQUIRE(true == true);
    }
  }
}
