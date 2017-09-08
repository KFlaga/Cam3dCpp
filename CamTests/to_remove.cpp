#include <gtest/gtest.h>

#include <boost/fusion/sequence.hpp>
#include <boost/fusion/include/sequence.hpp>

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
}
