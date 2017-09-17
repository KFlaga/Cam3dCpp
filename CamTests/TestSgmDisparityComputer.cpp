#include <CamImageProcessing/SgmDisparityComputer.hpp>
#include <CamImageProcessing/GreyScaleImage.hpp>
#include <gtest\gtest.h>

namespace cam3d
{

class TestSgmDisparityComputer : public ::testing::Test
{
public:
    static constexpr int imgSize = 10;
    static constexpr int last = imgSize - 1;

    using Image = GreyScaleImage;
    using CostComputer = CensusCostComputer<Image, BitWord32<8>>;
    using DisparityComputer = SgmDisparityComputer<CostComputer>;
    Image imageBase;
    Image imageMatched;
    CostComputer costComp;

    class TestDisparityComputer : public DisparityComputer
    {
    public:
        TestDisparityComputer(int rows, int cols, bool left, Image& imageBase, Image& imageMatched, CostComputer& costComp) :
            DisparityComputer(rows, cols, left, imageBase, imageMatched, costComp)
        { }

        Disparity findBestDisparity(Point2 pixelBase)
        {
            return DisparityComputer::findBestDisparity(pixelBase);
        }

        double findConfidence(int count, double cost)
        {
            return DisparityComputer::findConfidence(count, cost);
        }

        double findMean_Simple(int start, int count)
        {
            return DisparityComputer::findMean_Simple(start, count);
        }

        double findMean_Weighted(int start, int count)
        {
            return DisparityComputer::findMean_Weighted(start, count);
        }

        double findMean_WeightedPath(int start, int count)
        {
            return DisparityComputer::findMean_WeightedPath(start, count);
        }

        double findCost_Simple(double mean, int start, int count)
        {
            return DisparityComputer::findCost_Simple(mean, start, count);
        }

        double findCost_Squared(double mean, int start, int count)
        {
            return DisparityComputer::findCost_Squared(mean, start, count);
        }

        double findCost_Root(double mean, int start, int count)
        {
            return DisparityComputer::findCost_Root(mean, start, count);
        }
    } dispComp;

    TestSgmDisparityComputer() :
        imageBase{imgSize, imgSize},
        imageMatched{imgSize, imgSize},
        costComp{imgSize, imgSize},
        dispComp{imgSize, imgSize, true, imageBase, imageMatched, costComp}
    { }
};

TEST_F(TestSgmDisparityComputer, CanBeCreated)
{
    ASSERT_TRUE(true);
}

TEST_F(TestSgmDisparityComputer, CanAddDisparities)
{
    for(int i = 0; i < 8; ++i)
    {
        dispComp.storeDisparity(Disparity{i, Disparity::Valid, i, i, 1.0});
    }
}

TEST_F(TestSgmDisparityComputer, CanFinalize)
{
    for(int i = 0; i < 8; ++i)
    {
        dispComp.storeDisparity(Disparity{i, Disparity::Valid, i, i, 1.0});
    }
    dispComp.finalizeForPixel({1, 1});
}
}
