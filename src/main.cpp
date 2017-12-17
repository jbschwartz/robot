#include "../include/quaternion.hpp"
#include "../include/dual.hpp"
#include "../include/transform.hpp"

#include <iostream>

using rbt::Transform;
using rbt::Vector3;

int main() {
	auto t = Transform(Vector3({1, 0, 0}), 180.f, Vector3({4, 2, 6}));
	std::cout << t << std::endl;

	auto v = Vector3({3, 4, 5});

	auto result = t(v);

	std::cout << result << std::endl;
}
