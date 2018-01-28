#include "third_party/catch.hpp"
#include "vector.hpp"

using rbt::Real;
using rbt::Vector3;

TEST_CASE("Vector") {
  const auto v1 = Vector3();
  const auto v2 = Vector3({-1, 2, -3});
  const auto v3 = Vector3({ 2, -5, 3 });

  SECTION("defaults to zero vector") {
    REQUIRE(v1[0] == 0);
    REQUIRE(v1[1] == 0);
    REQUIRE(v1[2] == 0);
  }

  SECTION("equality operator") {
    REQUIRE(v1 == v1);
    REQUIRE_FALSE(v1 == v2);
  }

  SECTION("scalar multiplication") {
    const Real s = 3;
    REQUIRE(s * v2 == Vector3({-3, 6, -9}));
    REQUIRE(v2 * s == Vector3({-3, 6, -9}));
  }

  SECTION("scalar division") {
    const Real s = 2;
    REQUIRE(v2 / s == Vector3({-0.5, 1, -1.5}));
  }

  SECTION("vector subtraction") {
    const auto result = v1 - v2;
    const auto expected = Vector3({ 1, -2, 3 });

    REQUIRE(result == expected);
  }

  SECTION("dot") {
    const auto result = v2 * v3;
    const auto expected = -21;

    REQUIRE(result == expected);

    SECTION("is commutative") {
      const auto commuted = v3 * v2;
      REQUIRE(commuted == result);
    }
  }

}
