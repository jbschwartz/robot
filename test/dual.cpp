#include "third_party/catch.hpp"
#include "../include/dual.h"
#include "../include/quaternion.h"

using rbt::Dual;
using rbt::Real;
using rbt::Quaternion;

TEST_CASE("Dual") {
  SECTION("Number") {
    const auto a = Dual<Real>(1, 2);

    SECTION("conjugate") {
      const auto c = conjugate(a);
      REQUIRE(c == Dual<Real>(1, -2));
    }

    SECTION("norm") {
      const auto n = a.norm();
      REQUIRE(n == a * conjugate(a));
    }
  }

  SECTION("Quaternions") {
    const auto r1 = Quaternion(1, 2, 3, 4);
    const auto d1 = Quaternion(0, 1, 2, 3);

    const auto r2 = Quaternion(0, -1, 0, 0);
    const auto d2 = Quaternion(1, 3, -4, 5);

    const auto q1 = Dual<Quaternion>(r1, d1);
    const auto q2 = Dual<Quaternion>(r2, d2);

    SECTION("add") {
      const auto result = q1 + q2;
      const auto expected = Dual<Quaternion>(r1 + r2, d1 + d2);

      REQUIRE(result.r == expected.r);
      REQUIRE(result.d == expected.d);
    }

    SECTION("multiply") {
      const auto result = q1 * q2;
      const auto expected = Dual<Quaternion>(r1 * r2, r1 * d2 + d1 * r2);

      REQUIRE(result.r == expected.r);
      REQUIRE(result.d == expected.d);
    }

    SECTION("scalar multiply") {
      const Real s = 4;

      const auto resultPre = s * q1;
      const auto expected = Dual<Quaternion>(s * r1, s * d1);

      REQUIRE(resultPre.r == expected.r);
      REQUIRE(resultPre.d == expected.d);

      const auto resultPost = q1 * s;
      REQUIRE(resultPost.r == expected.r);
      REQUIRE(resultPost.d == expected.d);
    }

    SECTION("equality") {
      const auto result = Dual<Quaternion>(r1, d1);
      const auto expected = q1;

      REQUIRE(result == expected);
    }

    SECTION("conjugate") {
      const auto result = conjugate(q1);
      const auto expected = Dual<Quaternion>(conjugate(r1), -conjugate(d1));

      REQUIRE(result == expected);
    }

    SECTION("norm") {
      const auto result = q1.norm();
      const auto expected = q1 * conjugate(q1);

      REQUIRE(result == expected);
    }
  }
}
