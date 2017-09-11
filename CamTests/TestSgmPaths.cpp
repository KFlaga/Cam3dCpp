#include <CamImageProcessing/SgmPath.hpp>
#include <gtest\gtest.h>

namespace cam3d
{
static constexpr Point2 startPoint{5, 5};
class TestSgmPaths : public ::testing::Test
{
public:
    static constexpr int imgSize = 10;

    template<typename PathT>
    PathT preparePath(Point2 sp = startPoint)
    {
        PathT path;
        path.imageWidth = imgSize;
        path.imageHeight = imgSize;
        path.startPixel = sp;
        path.length = imgSize * 2;
        path.init();
        return path;
    }

    template<typename PathT>
    void testPath(PathT& path,
                  int expectedLength,
                  Point2 borderPixel,
                  Point2 finishPixel,
                  Point2 previousPixel,
                  Point2 sp = startPoint)
    {
        EXPECT_EQ(0, path.currentIndex);
        EXPECT_EQ(sp, path.currentPixel);
        EXPECT_EQ(path.currentPixel, path.previousPixel);
        EXPECT_EQ(expectedLength, path.length);
        EXPECT_EQ(borderPixel, PathT::getBorderPixel(startPoint, imgSize, imgSize));

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
    auto path = preparePath<ConcreteSgmPath<0, 1>>();
    int expectedLength = 5;
    Point2 border = {startPoint.y, 0};
    Point2 finish = {startPoint.y, imgSize - 1};
    Point2 previous = finish - Point2{ 0, 1 };
    testPath(path, expectedLength, border, finish, previous);
}

TEST_F(TestSgmPaths, PathWorks_Xm1)
{
    auto path = preparePath<ConcreteSgmPath<0, -1>>();
    int expectedLength = 6;
    Point2 border = {startPoint.y, imgSize - 1};
    Point2 finish = {startPoint.y, 0};
    Point2 previous = finish - Point2{ 0, -1 };
    testPath(path, expectedLength, border, finish, previous);
}

TEST_F(TestSgmPaths, PathWorks_Yp1)
{
    auto path = preparePath<ConcreteSgmPath<1, 0>>();
    int expectedLength = 5;
    Point2 border = {0, startPoint.x};
    Point2 finish = {imgSize - 1, startPoint.x};
    Point2 previous = finish - Point2{ 1, 0 };
    testPath(path, expectedLength, border, finish, previous);
}

TEST_F(TestSgmPaths, PathWorks_Ym1)
{
    auto path = preparePath<ConcreteSgmPath<-1, 0>>();
    int expectedLength = 6;
    Point2 border = {imgSize - 1, startPoint.x};
    Point2 finish = {0, startPoint.x};
    Point2 previous = finish - Point2{ -1, 0 };
    testPath(path, expectedLength, border, finish, previous);
}

TEST_F(TestSgmPaths, PathWorks_Xp1Yp1)
{
    auto path = preparePath<ConcreteSgmPath<1, 1>>();
    int expectedLength = 5;
    Point2 border = {0, 0};
    Point2 finish = {imgSize - 1, imgSize - 1};
    Point2 previous = finish - Point2{ 1, 1 };
    testPath(path, expectedLength, border, finish, previous);
}

TEST_F(TestSgmPaths, PathWorks_Xm1Yp1)
{
    auto path = preparePath<ConcreteSgmPath<1, -1>>();
    int expectedLength = 5;
    Point2 border = {1, imgSize - 1};
    Point2 finish = {imgSize - 1, 1};
    Point2 previous = finish - Point2{ 1, -1 };
    testPath(path, expectedLength, border, finish, previous);
}

TEST_F(TestSgmPaths, PathWorks_Xm1Ym1)
{
    auto path = preparePath<ConcreteSgmPath<-1, -1>>();
    int expectedLength = 6;
    Point2 border = {imgSize - 1, imgSize - 1};
    Point2 finish = {0, 0};
    Point2 previous = finish - Point2{ -1, -1 };
    testPath(path, expectedLength, border, finish, previous);
}

//TEST_F(TestSgmPaths, PathWorks_Xp1Yp2)
//{
//    auto path = preparePath<ConcreteSgmPath<2, 1>>();
//    int expectedLength = 5;
//    Point2 border = {2, 0};
//    Point2 finish = {7, imgSize - 1};
//    Point2 previous = finish - Point2{ 1, 1 };
//    testPath(path, expectedLength, border, finish, previous);
//}

//TEST_F(TestSgmPaths, PathWorks_Xp1Yp2_shortY)
//{
//    Point2 startPixel = {7, 3};
//    auto path = preparePath<ConcreteSgmPath<2, 1>>(startPixel);
//    int expectedLength = 6;
//    Point2 border = {2, 0};
//    Point2 finish = {imgSize - 1, 8};
//    Point2 previous = finish - Point2{ 0, 1 };
//    testPath(path, expectedLength, border, finish, previous, startPixel);
//}

//TEST_F(TestSgmPaths, PathWorks_Xm1Yp2)
//{
//    auto path = preparePath<ConcreteSgmPath<2, -1>>();
//    int expectedLength = 6;
//    Point2 border = {3, imgSize - 1};
//    Point2 finish = {7, 0};
//    Point2 previous = finish - Point2{ 0, -1 };
//    testPath(path, expectedLength, border, finish, previous);
//}

//TEST_F(TestSgmPaths, PathWorks_Xm2Yp1)
//{
//    auto path = preparePath<ConcreteSgmPath<1, -2>>();
//    int expectedLength = 5;
//    Point2 border = {0, 2};
//    Point2 finish = {imgSize - 1, 3};
//    Point2 previous = finish - Point2{ 1, -1 };
//    testPath(path, expectedLength, border, finish, previous);
//}

//TEST_F(TestSgmPaths, PathWorks_Xm2Yp1_shortX)
//{
//    Point2 startPixel = {3, 2};
//    auto path = preparePath<ConcreteSgmPath<1, -2>>(startPixel);
//    int expectedLength = 6;
//    Point2 border = {0, 3};
//    Point2 finish = {8, 0};
//    Point2 previous = finish - Point2{ 1, 0 };
//    testPath(path, expectedLength, border, finish, previous, startPixel);
//}
}
