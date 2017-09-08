#include <CamImageProcessing/SgmPath.hpp>
#include <gtest\gtest.h>

namespace cam3d
{
class TestSgmPaths : public ::testing::Test
{
public:
    static constexpr int imgSize = 10;
    Point2 startPoint = Point2{5, 5};

    template<typename PathT>
    PathT preparePath()
    {
        PathT path;
        path.imageWidth = imgSize;
        path.imageHeight = imgSize;
        path.startPixel = startPoint;
        path.length = imgSize * 2;
        path.init();
        return path;
    }

    template<typename PathT>
    void testPath(PathT& path,
                  int expectedLength,
                  Point2 borderPixel,
                  Point2 finishPixel,
                  Point2 previousPixel)
    {
        EXPECT_EQ(0, path.currentIndex);
        EXPECT_EQ(startPoint, path.currentPixel);
        EXPECT_EQ(path.currentPixel, path.previousPixel);
        EXPECT_EQ(expectedLength, path.length);
        EXPECT_EQ(borderPixel, PathT::GetBorderPixel(startPoint, imgSize, imgSize));

        while(path.haveNextPixel())
        {
            path.next();
        }

        EXPECT_EQ(path.length - 1, path.currentIndex);
        EXPECT_EQ(finishPixel, path.currentPixel);
        EXPECT_EQ(previousPixel, path.previousPixel);
    }
};

TEST_F(TestSgmPaths, PathWorks_Xp1)
{
    auto path = preparePath<Path_Str_XPos>();
    int expectedLength = 5;
    Point2 border = {startPoint.y, 0};
    Point2 finish = {startPoint.y, imgSize - 1};
    Point2 previous = finish - Point2{ 0, 1 };
    testPath(path, expectedLength, border, finish, previous);
}

TEST_F(TestSgmPaths, PathWorks_Xm1)
{
    auto path = preparePath<Path_Str_XNeg>();
    int expectedLength = 6;
    Point2 border = {startPoint.y, imgSize - 1};
    Point2 finish = {startPoint.y, 0};
    Point2 previous = finish - Point2{ 0, -1 };
    testPath(path, expectedLength, border, finish, previous);
}

TEST_F(TestSgmPaths, PathWorks_Yp1)
{
    auto path = preparePath<Path_Str_YPos>();
    int expectedLength = 5;
    Point2 border = {0, startPoint.x};
    Point2 finish = {imgSize - 1, startPoint.x};
    Point2 previous = finish - Point2{ 1, 0 };
    testPath(path, expectedLength, border, finish, previous);
}


TEST_F(TestSgmPaths, PathWorks_Ym1)
{
    auto path = preparePath<Path_Str_YNeg>();
    int expectedLength = 6;
    Point2 border = {imgSize - 1, startPoint.x};
    Point2 finish = {0, startPoint.x};
    Point2 previous = finish - Point2{ -1, 0 };
    testPath(path, expectedLength, border, finish, previous);
}
}
