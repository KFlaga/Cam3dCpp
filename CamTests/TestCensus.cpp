#include <CamImageProcessing/CensusCostComputer.hpp>
#include <CamImageProcessing/GreyScaleImage.hpp>
#include <CamCommon/BitWord.hpp>
#include <gtest\gtest.h>

namespace cam3d
{
class TestCensus32Grey : public ::testing::Test
{
public:
    static constexpr int imgSize = 10;
    static constexpr int maskSize = 1;
    static constexpr int maskLength = (maskSize + 1) * (maskSize + 1);
    using BitWord = BitWord32<maskLength / 32 + 1>;
    using Image = GreyScaleImage<imgSize, imgSize>;
    using Census = CensusCostComputer<GreyScaleImage<imgSize, imgSize>, BitWord>;

    Image baseImage;
    Image matchedImage;

    void prepareTestImagesSame()
    {
        // Images:
        // base: zeros in upper half, ones in lower half
        // matched: zeros in upper half, ones in lower half
        for(int y = 0; y < imgSize; ++y)
        {
            for(int x = 0; x < imgSize; ++x)
            {
                baseImage(y, x) = y < imgSize / 2 ? 0.0 : 1.0;
                matchedImage(y, x) = y < imgSize / 2 ? 0.0 : 1.0;
            }
        }
    }

    void prepareTestImagesDiffers()
    {
        // Images:
        // base: zeros in upper half, ones in lower half
        // matched: zeros in upper half, ones in lower half + halfs on y = imgSize/2
        for(int y = 0; y < imgSize; ++y)
        {
            for(int x = 0; x < imgSize; ++x)
            {
                baseImage(y, x) = y < imgSize / 2 ? 0.0 : 1.0;
                matchedImage(y, x) = y < imgSize / 2 ? 0.0 : (y > imgSize / 2 ? 1.0 : 0.5);
            }
        }
    }
};

TEST_F(TestCensus32Grey, CensusComputedCorrectlyBase)
{
    Census c{};
    c.setMaskHeight(maskSize);
    c.setMaskWidth(maskSize);
    prepareTestImagesSame();
    c.init(baseImage, matchedImage);

    uint32_t mask0[1] = { 0 };
    BitWord bw0{mask0};
    EXPECT_EQ(0u, bw0.getHammingDistance(c.getCensusBase()(3, 3)));
    EXPECT_EQ(0u, bw0.getHammingDistance(c.getCensusBase()(7, 3)));
    EXPECT_EQ(3u, bw0.getHammingDistance(c.getCensusBase()(5, 3)));
    EXPECT_EQ(0u, bw0.getHammingDistance(c.getCensusBase()(0, 3)));
    EXPECT_EQ(0u, bw0.getHammingDistance(c.getCensusBase()(0, 0)));
}

TEST_F(TestCensus32Grey, CensusComputedCorrectlyMatched)
{
    Census c{};
    c.setMaskHeight(maskSize);
    c.setMaskWidth(maskSize);
    prepareTestImagesDiffers();
    c.init(baseImage, matchedImage);

    uint32_t mask0[1] = { 0 };
    BitWord bw0{mask0};
    EXPECT_EQ(0u, bw0.getHammingDistance(c.getCensusMatched()(4, 3)));
    EXPECT_EQ(3u, bw0.getHammingDistance(c.getCensusMatched()(5, 3)));
    EXPECT_EQ(3u, bw0.getHammingDistance(c.getCensusMatched()(6, 3)));
}

TEST_F(TestCensus32Grey, CostComputedCorrectly)
{
    Census c{};
    c.setMaskHeight(maskSize);
    c.setMaskWidth(maskSize);
    prepareTestImagesDiffers();
    c.init(baseImage, matchedImage);

    EXPECT_DOUBLE_EQ(0.0, c.getCost({3, 3}, {3, 3}));
    EXPECT_DOUBLE_EQ(0.0, c.getCost({4, 3}, {4, 3}));
    EXPECT_DOUBLE_EQ(0.0, c.getCost({5, 3}, {5, 3}));
    EXPECT_DOUBLE_EQ(3.0, c.getCost({6, 3}, {6, 3}));
    EXPECT_DOUBLE_EQ(0.0, c.getCost({7, 3}, {7, 3}));
}
}
