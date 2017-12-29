#include "../include/quaternion.hpp"
#include "../include/dual.hpp"
#include "../include/serial.hpp"
#include "../include/joint.hpp"
#include "../include/transform.hpp"

#include <iostream>

using rbt::Transform;
using rbt::Vector3;
using rbt::Serial;
using rbt::Joint;

int main() {
	// ABB IRB120
	auto s = Serial({
		Joint(-90,	0, 		0, 		290),
    Joint(0,   	270,	-90, 	0),
    Joint(90,		-70, 	180,	0),
    Joint(-90, 	0,  	0, 		302),
    Joint(90, 	0, 		0, 		0),
    Joint(0,   	0, 		0,   	72)
	});

	auto result = s.pose({45, 45, 45, 45, 45, 45});

	std::cout << result.position() << std::endl;
	std::cout << result.orientation() << std::endl;
}
