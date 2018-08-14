#include "third_party/catch.hpp"
#include "utilities.hpp"

using rbt::toDegrees;
using rbt::toRadians;
using rbt::inchesToMillimeters;
using rbt::millimetersToInches;
using rbt::Real;
using rbt::sign;
using rbt::minusPiToPi;

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

  SECTION("sign") {
    SECTION("positive number") {
      const auto value = 10;
      const auto result = sign(value);
      const auto expected = 1;

      REQUIRE(result == expected);
    }

    SECTION("negative number") {
      const auto value = -10;
      const auto result = sign(value);
      const auto expected = -1;

      REQUIRE(result == expected);
    }

    SECTION("zero") {
      const auto value = 0;
      const auto result = sign(value);
      const auto expected = 0;

      REQUIRE(result == expected);
    }
  }

  SECTION("minusPiToPi") {
    const std::vector<Real> values    = { 30, -30, 190, -190, 750, -750 };
    const std::vector<Real> expecteds = { 30, -30, -170, 170, 30, -30 };

    for(unsigned int i = 0; i < values.size(); ++i) {
      REQUIRE(Approx(minusPiToPi(toRadians(values[i]))) == toRadians(expecteds[i]));
    }
  }
}
