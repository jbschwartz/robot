#include "../third_party/catch.hpp"
#include "../include/vector.hpp"

template <std::size_t N>
using Vector = rbt::Vector<rbt::Real, N>;

template <std::size_t N>
class VectorMatcher : public Catch::MatcherBase<Vector<N>>
{
  Vector<N> a;
public:
  VectorMatcher(const Vector<N>& a) : a(a) {};

  virtual bool match(const Vector<N>& b) const override {
    for(std::size_t i = 0; i < N; ++i) {
      if(Approx(a[i]) != b[i]) return false;
    }
    return true;
  }

  virtual std::string describe() const override {
      std::ostringstream ss;
      ss << "is equal to { ";
      for(std::size_t i = 0; i < N; ++i) {
        ss << a[i] << " ";
      }
      ss << "}";
      return ss.str();
  }
};

template <std::size_t N>
inline VectorMatcher<N> ComponentsEqual(const Vector<N>& a) {
  return VectorMatcher<N>(a);
}
