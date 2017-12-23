#include "third_party/catch.hpp"
#include "utilities.hpp"

using rbt::toDegrees;
using rbt::toRadians;
using rbt::inchesToMillimeters;
using rbt::millimetersToInches;
using rbt::Real;

TEST_CASE("Utilities") {
  const Real radians = 3 * rbt::PI / 4;
  const Real degrees = 135;
  const Real inches = 2;
  const Real millimeters = 50.8;

  SECTION("toDegrees") {
    REQUIRE(Approx(toDegrees(radians)) == degrees);
  }

  SECTION("toRadians") {
    REQUIRE(Approx(toRadians(degrees)) == radians);
  }

  SECTION("inchesToMillimeters") {
    REQUIRE(Approx(inchesToMillimeters(inches)) == millimeters);
  }

  SECTION("millimetersToInches") {
    REQUIRE(Approx(millimetersToInches(millimeters)) == inches);
  }

}
