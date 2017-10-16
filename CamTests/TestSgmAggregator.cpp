#include <CamImageProcessing/SgmCostAggregatorForTests.hpp>
#include <CamImageProcessing/GreyScaleImage.hpp>
#include <gtest\gtest.h>
#include <random>

namespace cam3d
{

class TestSgmAggregator : public ::testing::Test
{
public:
    static constexpr int imgSize = 20;
    static constexpr int last = imgSize - 1;
    static constexpr int bitwordLength = 1;
    static constexpr int maskRadius = 2;
    static constexpr int patchSize = 4;
    int randSeed = 1000;

    using Image = GreyScaleImage;
    using BitWord = BitWord32<bitwordLength>;
    using Census = CensusCostComputer<GreyScaleImage, BitWord>;
    using Sgm = SgmCostAggregatorForTests<Image, Census>;

    Image imageBase;
    Image imageMatched;
    Sgm agg;
    DisparityMap map;
    SgmParameters params;

    TestSgmAggregator() :
        imageBase{imgSize, imgSize},
        imageMatched{imgSize, imgSize},
        map{imgSize, imgSize},
        agg{imgSize, imgSize, true, imageBase, imageMatched, map}
    {

    }

    SgmParameters prepareParams()
    {
        SgmParameters params;

        params.censusMaskRadius = maskRadius;
        params.lowPenaltyCoeff = 0.02;
        params.highPenaltyCoeff = 0.04;
        params.gradientCoeff = 0.0;
        params.disparityMeanMethod = MeanMethod::WeightedAverage;
        params.disparityCostMethod = CostMethod::DistanceToMean;
        params.diparityPathLengthThreshold = 1.0;

        return params;
    }

    void prepareNoiseTestImages()
    {
        std::srand(randSeed);

        // Get random noise images
        for(int y = 0; y < imgSize; ++y)
        {
            for(int x = 0; x < imgSize; ++x)
            {
                imageBase(y, x) = getRandomSample();
                imageMatched(y, x) = getRandomSample();
            }
        }

        // Shift matched image by 0 to -4, increasing each 3 rows
        for(int d = 0; d < imgSize / patchSize; ++d)
        {
            for(int y = d * patchSize; y < (d+1) * patchSize; ++y)
            {
                for(int x = 0; x < imgSize; ++x)
                {
                    if(x - d >= 0)
                    {
                        imageMatched(y, x - d) = imageBase(y, x);
                    }
                }
            }
        }
    }

    double getRandomSample()
    {
        int sample = std::rand() % 1000;
        return ((double)sample) / 1000.0;
    }
};

TEST_F(TestSgmAggregator, CanBeCreated)
{
    ASSERT_TRUE(true);
}

TEST_F(TestSgmAggregator, NoiseImageShift)
{
    prepareNoiseTestImages();
    Sgm sgm{imgSize, imgSize, true, imageBase, imageMatched, map};
    sgm.computeMatchingCosts(prepareParams());

    int matches = 0;
    int trueNegatives = 0;
    int truePositives = 0;
    int falseNegatives = 0;
    int falsePositives = 0;
    int falseNegatives2 = 0;
    int falsePositives2 = 0;
    double tc = 0.55;
    auto map = sgm.getDisparityMap();
    for(int d = 0; d < imgSize / patchSize; ++d)
    {
        for(int y = d * patchSize; y < (d+1) * patchSize; ++y)
        {
            for(int x = 0; x < imgSize; ++x)
            {
                if(x - d >= 0)
                {
                    // Expected high confidence / low cost correct match
                    // May be few low confid ence bad match
                    int dact = map(y,x).dx;
                    if(dact == -d)
                    {
                        ++matches;
                        if( map(y, x).confidence > tc )
                        {
                            ++truePositives;
                        }
                        else
                        {
                            ++falseNegatives;
                        }
                    }
                    else
                    {
                        // Expected low confidence / high cost match
                        if( map(y, x).confidence > tc )
                        {
                            ++falsePositives;
                        }
                        else
                        {
                            ++falseNegatives2;
                        }
                    }
                    // EXPECT_EQ(-d, dact);
                }
                else
                {
                    if( map(y, x).confidence > tc )
                    {
                        ++falsePositives2;
                    }
                    else
                    {
                        ++trueNegatives;
                    }
                }
            }
        }
    }
    int n = imgSize / patchSize - 1;
    ASSERT_EQ(imgSize * imgSize - patchSize * n * (n+1) / 2, matches);
}
}
