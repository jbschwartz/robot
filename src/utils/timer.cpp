#include "utils/timer.hpp"

#include <iostream>

namespace rbt
{

using namespace std::chrono;

Timer::Timer(const std::string& label) : label(label) {
  this->begin = steady_clock::now();
}

Timer::~Timer() {
  auto elapsed = duration_cast<milliseconds>(steady_clock::now() - this->begin).count();
  std::cout << this->label << " elasped time: " << elapsed << " [ms]" << std::endl;
}

}
