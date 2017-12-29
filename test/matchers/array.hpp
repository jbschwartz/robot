#include "../third_party/catch.hpp"

#include <iostream>

template <typename T, std::size_t N>
class ArrayMatcher : public Catch::MatcherBase<std::array<T,N>>
{
  std::array<T, N> a;
public:
  ArrayMatcher(const std::array<T, N>& a) : a(a) {};

  virtual bool match(const std::array<T, N>& b) const override {
    auto bElement = b.begin();

    for(const T& aElement : this->a) {
      if(Approx(aElement) != *bElement++) return false;
    }
    return true;
  }

  virtual std::string describe() const {
      std::ostringstream ss;
      ss << "is not equal to";
      return ss.str();
  }
};

template <typename T, std::size_t N>
inline ArrayMatcher<T, N> ComponentsEqual(const std::array<T, N>& a) {
  return ArrayMatcher<T, N>(a);
}
