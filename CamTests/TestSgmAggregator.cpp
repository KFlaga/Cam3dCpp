#include <CamImageProcessing/SgmCostAggregator.hpp>
#include <CamImageProcessing/GreyScaleImage.hpp>
#include <gtest\gtest.h>

namespace cam3d
{

class TestSgmAggregator : public ::testing::Test
{
public:
    static constexpr int imgSize = 10;
    static constexpr int last = imgSize - 1;

    using Image = GreyScaleImage;
    Image imageBase;
    Image imageMatched;
    SgmCostAggregator<Image> agg;

    TestSgmAggregator() :
        imageBase{imgSize, imgSize},
        imageMatched{imgSize, imgSize},
        agg{imgSize, imgSize, true}
    { }
};

TEST_F(TestSgmAggregator, CanBeCreated)
{
    ASSERT_TRUE(true);
}

TEST_F(TestSgmAggregator, CanBeInited)
{
    agg.computeMatchingCosts(imageBase, imageMatched);
}
}
