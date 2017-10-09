#include <CamImageProcessing/SgmDisparityComputerForTests.hpp>
#include <CamImageProcessing/GreyScaleImage.hpp>
#include <gtest\gtest.h>

namespace cam3d
{

class TestSgmDisparityComputer : public ::testing::Test
{
public:
    static constexpr int imgSize = 10;
    static constexpr int last = imgSize - 1;
    static constexpr int maskRadius = 1;

    using Image = GreyScaleImage;
    using BitWord = BitWord32<maskRadius>;
    using CostComputer = CensusCostComputer<Image, BitWord>;
    using DisparityComputer = SgmDisparityComputerForTests<Image, CostComputer>;
    Image imageBase;
    Image imageMatched;
    DisparityMap map;
    CostComputer costComp;

    class TestDisparityComputer : public DisparityComputer
    {
    public:
        TestDisparityComputer(int rows, int cols, DisparityMap& map, Image& imageBase, Image& imageMatched, CostComputer& costComp) :
            DisparityComputer(rows, cols, map, imageBase, imageMatched, costComp)
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
        map{imgSize, imgSize},
        costComp{imgSize, imgSize},
        dispComp{imgSize, imgSize, map, imageBase, imageMatched, costComp}
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

TEST_F(TestSgmDisparityComputer, FindMeanOk)
{
    int count = 8;
    for(int i = 0; i < count; ++i)
    {
        dispComp.storeDisparity(Disparity{-i, Disparity::Valid, -i, i, 1.0});
    }
    double meanSimple = -3.5;
    double meanWeighted = -1.9434954007884362680683311432326;

    EXPECT_DOUBLE_EQ(meanSimple, dispComp.findMean_Simple(0, count));
    EXPECT_DOUBLE_EQ(meanWeighted, dispComp.findMean_Weighted(0, count));
}

TEST_F(TestSgmDisparityComputer, FindCostOk)
{
    int count = 8;
    for(int i = 0; i < count; ++i)
    {
        dispComp.storeDisparity(Disparity{i, Disparity::Valid, i, i, 1.0});
    }
    double meanSimple = 3.5;
    double costSimple = 0.25;
    double costSquared = 0.01025390625;
    double costRoot = 0.7071067811865475;

    EXPECT_DOUBLE_EQ(costSimple, dispComp.findCost_Simple(meanSimple, 0, count));
    EXPECT_DOUBLE_EQ(costSquared, dispComp.findCost_Squared(meanSimple, 0, count));
    EXPECT_DOUBLE_EQ(costRoot, dispComp.findCost_Root(meanSimple, 0, count));
}

TEST_F(TestSgmDisparityComputer, FindsExpectedBestDisparity)
{
    int count = 8;
    for(int i = 0; i < count; ++i)
    {
        dispComp.storeDisparity(Disparity{i, Disparity::Valid, i, i, 1.0});
    }

    dispComp.setMeanMethod(MeanMethod::SimpleAverage);
    dispComp.setCostMethod(CostMethod::DistanceToMean);

    dispComp.finalizeForPixel(Point2{2, 2});
    Disparity d = dispComp.getDisparityMap()(Point2{2, 2});

    EXPECT_GT(0.0, d.subDx);

    for(int i = 0; i < count; ++i)
    {
        dispComp.storeDisparity(Disparity{i, Disparity::Valid, i, i, 1.0});
    }

    dispComp.setMeanMethod(MeanMethod::WeightedAverage);
    dispComp.setCostMethod(CostMethod::DistanceToMean);

    dispComp.finalizeForPixel(Point2{3, 3});
    d = dispComp.getDisparityMap()(Point2{3, 3});

    EXPECT_GT(0.0, d.subDx);
}
}
