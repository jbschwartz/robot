#include "../third_party/catch.hpp"
#include "../matchers/angles.hpp"
#include "../../include/ik.hpp"
#include "../../include/utilities.hpp"
#include "../../include/typedefs.hpp"

using namespace rbt;
using namespace rbt::ik;

TEST_CASE("solveWaist") {
  const std::vector<Real> angles = { 0, 30, 90, 150, -120, -90, -30 };
  const Real armLength = 3;

  SECTION("calculates waist angles (-PI, PI] with and without shoulder offset") {
    // Solve with and without a shoulder offset:
    //    Prescribe the alpha angle and solve for shoulderOffset
    //    This makes the expected results easy to reason about
    const std::vector<Real> alphas = { 0, 15, -15 };

    for(auto alpha : alphas) {
      alpha = toRadians(alpha);
      const Real shoulderOffset = armLength * std::sin(alpha);

      for(auto angle : angles) {
        angle = toRadians(angle);
        // Calculate target position
        const Real y = armLength * sin(angle);
        const Real x = armLength * cos(angle);

        const auto result = solveWaist(x, y, 0, shoulderOffset);
        const std::vector<Real> expected = {
          minusPiToPi(angle - alpha),
          minusPiToPi(angle + alpha + PI)
        };

        CHECK_THAT(result, ComponentsEqual(expected));
      }
    }
  }

  SECTION("determines the singular position") {
    const auto result = solveWaist(0, 0);
    const auto expected = Angles({ SINGULAR });

    CHECK_THAT(result, ComponentsEqual(expected));
  }
}
