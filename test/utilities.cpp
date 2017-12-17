#include "third_party/catch.hpp"
#include "utilities.h"

using rbt::toDegrees;
using rbt::toRadians;
using rbt::Real;

TEST_CASE("Utilities") {
  const Real radians = 3 * rbt::PI / 4;
  const Real degrees = 135;

  SECTION("toDegrees") {
    REQUIRE(Approx(toDegrees(radians)) == degrees);
  }

  SECTION("toRadians") {
    REQUIRE(Approx(toRadians(degrees)) == radians);
  }
}
