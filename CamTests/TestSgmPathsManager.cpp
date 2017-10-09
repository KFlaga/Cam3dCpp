#include <CamImageProcessing/SgmPathsManager.hpp>
#include <gtest\gtest.h>

namespace cam3d
{

class TestSgmPathsManager : public ::testing::Test
{
public:
    static constexpr int imgSize = 10;
    static constexpr int last = imgSize - 1;

    SgmPathsManager mgr;

    TestSgmPathsManager() :
        mgr{imgSize, imgSize,[](Point2 a, Point2 b){ return 1.0; }, true}
    { }
};

TEST_F(TestSgmPathsManager, CanBeCreated)
{
    ASSERT_TRUE(true);
}

TEST_F(TestSgmPathsManager, CreatesBorderPaths)
{
    mgr.init();
    using Dir = PathDirection;
    ASSERT_TRUE( mgr.getPath({last/2, last/2}, Dir::NegX) == nullptr );
    ASSERT_TRUE( mgr.getPath({0, 0}, Dir::PosX) != nullptr );
    ASSERT_TRUE( mgr.getPath({0, 0}, Dir::NegX) == nullptr );
    ASSERT_TRUE( mgr.getPath({last, last}, Dir::NegX) != nullptr );
    ASSERT_TRUE( mgr.getPath({last, last}, Dir::PosY) == nullptr );
    ASSERT_TRUE( mgr.getPath({last, last/2}, Dir::PosY) == nullptr );
    ASSERT_TRUE( mgr.getPath({last, last/2}, Dir::NegY) != nullptr );
    ASSERT_TRUE( mgr.getPath({last, last/2}, Dir::PosX) == nullptr );
    ASSERT_TRUE( mgr.getPath({last, last/2}, Dir::NegX) == nullptr );
    ASSERT_TRUE( mgr.getPath({last, last/2}, Dir::NegX_PosY) == nullptr );
    ASSERT_TRUE( mgr.getPath({last, last/2}, Dir::NegX_NegY) != nullptr );
    ASSERT_TRUE( mgr.getPath({last, last/2}, Dir::PosX_NegY) != nullptr );
    ASSERT_TRUE( mgr.getPath({last/2, last}, Dir::NegX) != nullptr );
    ASSERT_TRUE( mgr.getPath({last/2, last}, Dir::PosX) == nullptr );
    ASSERT_TRUE( mgr.getPath({last/2, last}, Dir::PosY) == nullptr );
    ASSERT_TRUE( mgr.getPath({last/2, last}, Dir::NegX_NegY) != nullptr );
    ASSERT_TRUE( mgr.getPath({last/2, last}, Dir::PosX_NegY) == nullptr );
}
}
