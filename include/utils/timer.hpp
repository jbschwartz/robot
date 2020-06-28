#ifndef __TIMER_HPP__
#define __TIMER_HPP__

#include <chrono>
#include <string>

namespace rbt
{

// A simple elapsed time timer for basic "profiling".
class Timer
{
public:
  Timer(const std::string& label);
  ~Timer();

private:
  // The time point at which the timer began.
  std::chrono::steady_clock::time_point begin;

  // The name of this timer.
  std::string label;
};

}

#endif /* __TIMER_HPP__ */
