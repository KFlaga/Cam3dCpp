#include <gtest/gtest.h>

#include <boost/fusion/sequence.hpp>
#include <boost/fusion/include/sequence.hpp>

#include <boost/hana.hpp>
#include <boost/any.hpp>
#include <cassert>
#include <iostream>
#include <string>
#include <typeindex>
namespace hana = boost::hana;

struct Fish { std::string name; };
struct Cat  { std::string name; };
struct Dog  { std::string name; };

namespace xxx
{
class TestFusion : public ::testing::Test
{

public:

};

TEST_F(TestFusion, a1)
{
    boost::fusion::vector<int, char, std::string> stuff{1, 'x', "abc"};
    EXPECT_EQ(1, boost::fusion::at_c<0>(stuff));
    EXPECT_EQ('x', boost::fusion::at_c<1>(stuff));
    EXPECT_STREQ("abc", boost::fusion::at_c<2>(stuff).c_str());
}

class TestHana : public ::testing::Test
{

public:

};

TEST_F(TestHana, a1)
{
    using namespace hana::literals;

    auto animals = hana::make_tuple(Fish{"fish"}, Cat{"cat"}, Dog{"dog"});
    Cat cat = animals[1_c];

    auto names = hana::transform(animals, [](auto a) {
        return a.name;
    });

    auto rnames = hana::reverse(names);
    auto expectedNames = hana::make_tuple("dog", "cat", "fish");

    ASSERT_TRUE( rnames == expectedNames );
}

template <typename T>
auto case_ = [](auto f)
{
    return hana::make_pair(hana::type_c<T>, f);
};

struct default_t;
auto default_ = case_<default_t>;

template <typename Any>
auto switch_(Any& a)
{
  return [&a](auto... cases_)
  {
      auto cases = hana::make_tuple(cases_...);

      auto defaultCase = hana::find_if(cases, [](auto const& c)
      {
         return hana::first(c) == hana::type_c<default_t>;
      });

      static_assert(defaultCase != hana::nothing, "switch is missing a default_ case");

      auto rest = hana::filter(cases, [](auto const& c)
      {
          return hana::first(c) != hana::type_c<default_t>;
      });

      return hana::unpack(rest, [&](auto& ...rest)
      {
          return process(a, a.type(), hana::second(*defaultCase), rest...);
      });
  };
}

template <typename Any, typename Default>
auto process(Any&, std::type_index const&, Default& defaultCase)
{
  return defaultCase();
}

template <typename Any, typename Default, typename Case, typename ...Rest>
auto process(Any& a, std::type_index const& t, Default& defaultCase,
             Case& case_, Rest& ...rest)
{
  using T = typename decltype(+hana::first(case_))::type;
  return t == typeid(T) ? hana::second(case_)(*boost::unsafe_any_cast<T>(&a))
                        : process(a, t, defaultCase, rest...);
}

TEST_F(TestHana, a2)
{
    boost::any a = 'x';
    auto r = switch_(a)(
      case_<int>([](auto) -> int { return 1; }),
      case_<char>([](auto) -> long { return 2l; }),
      default_([]() -> long long { return 3ll; })
    );
    // r is inferred to be a long long
    static_assert(std::is_same<decltype(r), long long>{}, "");
    ASSERT_TRUE(r == 2ll);
}

}
