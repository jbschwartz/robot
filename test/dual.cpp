#include "third_party/catch.hpp"
#include "../include/dual.h"
#include "../include/quaternion.h"

TEST_CASE("Dual Quaternions") {
  auto a = Dual<Quaternion>();
  auto b = Dual<Quaternion>(Quaternion(1, 2, 3, 4), Quaternion(0, 1, 2, 3));

  SECTION("add") {
    auto c = b + b;
  }
}
