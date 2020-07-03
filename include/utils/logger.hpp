#ifndef __LOGGER_HPP__
#define __LOGGER_HPP__

#include <sstream>
#include <string>

// Shorthand logging call.
#define LOG(x) Logger().out(LEVEL::x)

namespace rbt {

enum class LEVEL {
  INFO,
  WARN,
  ERROR
};

class Logger {
public:
  Logger();
  ~Logger();

  std::stringstream& out(const LEVEL& level);
private:
  std::stringstream os;

  std::string level_to_string(const LEVEL& level);
};

}

#endif /* __LOGGER_HPP__ */
