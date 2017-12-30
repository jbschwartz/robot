#include "third_party/catch.hpp"
#include "../include/quaternion.hpp"

#include <cmath>

using rbt::Real;
using rbt::Quaternion;
using rbt::norm;

TEST_CASE("Quaternions") {
  const auto q1 = Quaternion(1, 2, 3, 4);
  const auto q2 = Quaternion(4, -3, 2, -1);

  SECTION("defaults to identity") {
    const auto identity = Quaternion();

    REQUIRE(identity.r == 1);
    REQUIRE(identity.x == 0);
    REQUIRE(identity.y == 0);
    REQUIRE(identity.z == 0);

    SECTION("identity is multiplicative identity") {
      REQUIRE(identity * q1 == q1);
      REQUIRE(q1 * identity == q1);
    }
  }

  SECTION("construct in r, x, y, z order") {
    REQUIRE(q1.r == 1);
    REQUIRE(q1.x == 2);
    REQUIRE(q1.y == 3);
    REQUIRE(q1.z == 4);
  }

  SECTION("add") {
    const auto result = q1 + q2;
    const auto expected = Quaternion(5, -1, 5, 3);

    REQUIRE(result == expected);
  }

  SECTION("add assignment") {
    auto result = q1;
    result += q2;
    const auto expected = Quaternion(5, -1, 5, 3);

    REQUIRE(result == expected);
  }

  SECTION("subtract") {
    const auto result = q1 - q2;
    const auto expected = Quaternion(-3, 5, 1, 5);

    REQUIRE(result == expected);
  }

  SECTION("subtract assignment") {
    auto result = q1;
    result -= q2;
    const auto expected = Quaternion(-3, 5, 1, 5);

    REQUIRE(result == expected);
  }

  SECTION("multiply") {
    const auto result = q1 * q1;
    const auto expected = Quaternion(-28, 4, 6, 8);

    REQUIRE(result == expected);
  }

  SECTION("multiply assignment") {
    auto result = q1;
    result *= q1;
    const auto expected = Quaternion(-28, 4, 6, 8);

    REQUIRE(result == expected);
  }

  SECTION("scalar multiply") {
    const Real s = 4;

    const auto resultPre = s * q1;
    const auto expected = Quaternion(4, 8, 12, 16);

    REQUIRE(resultPre == expected);

    const auto resultPost = q1 * s;
    REQUIRE(resultPost == expected);
  }

  SECTION("scalar multiply assignment") {
    const Real s = 4;
    auto result = q1;
    result *= s;
    const auto expected = Quaternion(4, 8, 12, 16);

    REQUIRE(result == expected);
  }

  SECTION("scalar divide") {
    const Real s = 4;

    const auto result = q1 / s;
    const auto expected = Quaternion(0.25, 0.5, 0.75, 1);

    REQUIRE(result == expected);
  }

  SECTION("conjugate") {
    const auto result = conjugate(q1);
    const auto expected = Quaternion(1, -2, -3, -4);

    REQUIRE(result == expected);
  }

  SECTION("norm") {
    const auto result = norm(q1);
    const auto expected = std::sqrt(30);

    REQUIRE(result == Approx(expected));
  }

  SECTION("normalize") {
    const auto result = normalize(q1 * q1);
    const auto expected = Quaternion(-14/15.f, 2/15.f, 1/5.f, 4/15.f);

    REQUIRE(result == expected);
  }

  SECTION("unary minus (negate)") {
    const auto result = -q1;
    const auto expected = Quaternion(-1, -2, -3, -4);

    REQUIRE(result == expected);
  }
}
