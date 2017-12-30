#include "third_party/catch.hpp"
#include "../include/dual.hpp"
#include "../include/quaternion.hpp"
#include "../include/norm.hpp"

using rbt::Dual;
using rbt::Real;
using rbt::Quaternion;
using rbt::norm;

TEST_CASE("Dual") {
  SECTION("Number") {
    const auto a = Dual<Real>(1, 2);

    SECTION("conjugate") {
      const auto c = conjugate(a);
      REQUIRE(c == Dual<Real>(1, -2));
    }

    SECTION("norm") {
      const auto n = norm(a);
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

    SECTION("defaults to identity") {
      const auto result = Dual<Quaternion>();
      const auto expected = Dual<Quaternion>(Quaternion(1,0,0,0), Quaternion(0,0,0,0));

      REQUIRE(result == expected);

      SECTION("identity is multiplicative identity") {
        REQUIRE(q1 * result == q1);
        REQUIRE(result * q1 == q1);
      }
    }

    SECTION("add") {
      const auto result = q1 + q2;
      const auto expected = Dual<Quaternion>(r1 + r2, d1 + d2);

      REQUIRE(result == expected);
    }

    SECTION("add assignment") {
      auto result = q1;
      result += q2;
      const auto expected = Dual<Quaternion>(r1 + r2, d1 + d2);

      REQUIRE(result == expected);
    }

    SECTION("subtract") {
      const auto result = q1 - q2;
      const auto expected = Dual<Quaternion>(r1 - r2, d1 - d2);

      REQUIRE(result == expected);
    }

    SECTION("subtract assignment") {
      auto result = q1;
      result -= q2;
      const auto expected = Dual<Quaternion>(r1 - r2, d1 - d2);

      REQUIRE(result == expected);
    }

    SECTION("multiply") {
      const auto result = q1 * q2;
      const auto expected = Dual<Quaternion>(r1 * r2, r1 * d2 + d1 * r2);

      REQUIRE(result == expected);
    }

    SECTION("multiply assignment") {
      auto result = q1;
      result *= q2;
      const auto expected = Dual<Quaternion>(r1 * r2, r1 * d2 + d1 * r2);

      REQUIRE(result == expected);
    }


    SECTION("scalar multiply") {
      const Real s = 4;

      const auto resultPre = s * q1;
      const auto expected = Dual<Quaternion>(s * r1, s * d1);

      REQUIRE(resultPre == expected);

      const auto resultPost = q1 * s;
      REQUIRE(resultPost == expected);
    }

    SECTION("scalar divide") {
      const Real s = 4;

      const auto result = q1 / s;
      const auto expected = Dual<Quaternion>(r1 / s, d1 / s);

      REQUIRE(result == expected);
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
      const auto result = norm(q1);
      const auto expected = norm(q1.r);

      REQUIRE(result == expected);
    }
  }
}
