#include "utils/logger.hpp"

#include <iostream>
#include <sstream>

namespace rbt {

Logger::Logger() {}

Logger::~Logger() {
  std::cout << os.rdbuf() << std::endl;
}

std::stringstream& Logger::out(const LEVEL& level) {
  this->os << this->level_to_string(level);
  return this->os;
}

std::string Logger::level_to_string(const LEVEL& level) {
  switch(level) {
    case LEVEL::WARN:
      return {"\033[33m[WARN]\033[0m "};
    case LEVEL::ERROR:
      return {"\033[31m[ERROR]\033[0m "};
    default:
      return {};
  }
}

}
