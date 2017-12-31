#include "third_party/catch.hpp"
#include "matchers/angles.hpp"
#include "../include/ik.hpp"
#include "../include/utilities.hpp"
#include "../include/typedefs.hpp"

#include <algorithm>

using namespace rbt;
using namespace rbt::ik;

TEST_CASE("Inverse Kinematics") {
  SECTION("waistAngles") {
    SECTION("with no shoulder offset") {
      const Real angle = 30;
      const Real x = 3 * cos(toRadians(angle));
      const Real y = 3 * sin(toRadians(angle));

      SECTION("calculates waist angles in radians") {
        const auto result = waistAngles(x, y);
        const auto expected = Angles({
          toRadians(angle),
          toRadians(angle + 180)
        });

        CHECK_THAT(result, ComponentsEqual(expected));
      }

      SECTION("appropriately determines singular position") {
        const auto result = waistAngles(0, 0);
        const auto expected = Angles({ SINGULAR });

        CHECK_THAT(result, ComponentsEqual(expected));
      }
    }

    SECTION("with shoulder offset") {
      const Real x = 0;
      const Real y = 5;
      const Real offset = 2;

      SECTION("calculates waist angles in radians") {
        const auto result = waistAngles(x, y, offset);

        // This calculation is a special case when x = 0:
        //            ^ Y
        //            |
        //            X <--- (0, Y)
        //          / |
        //        /   |
        //  -\--- \   |
        // offset  \  |
        //     \    \-| <--- Angle = acos(offset / Y)
        //     _\___ \|
        // -----------X--(0,0)------> X
        const auto expected = Angles({
          std::acos(offset / y),
          std::asin(offset / y) + toRadians(270)
        });

        CHECK_THAT(result, ComponentsEqual(expected));
      }
    }
  }
}
