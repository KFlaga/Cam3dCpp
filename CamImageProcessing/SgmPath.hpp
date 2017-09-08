#pragma once

#include <CamCommon/Vector2.hpp>

namespace cam3d
{
enum class PathDirection : int
{
    PosX, NegX, PosY, NegY,
    PosX_PosY, NegX_PosY, PosX_NegY, NegX_NegY,
    PosX2_PosY, NegX2_PosY, PosX2_NegY, NegX2_NegY,
    PosX_PosY2, NegX_PosY2, PosX_NegY2, NegX_NegY2,
};

class SgmPath
{
public:
    int imageWidth;
    int imageHeight;
    int length;

    Point2 startPixel;
    Point2 currentPixel;
    Point2 previousPixel;
    int currentIndex;

    bool haveNextPixel()
    {
        return currentIndex < length - 1;
    }

    double* lastStepCosts; // Needs to be allocated externally

    virtual void init() = 0;
    virtual void next() = 0;

    using BorderPixelGetter = Point2(Point2, int, int);
};

template<int X, int Y>
class ConcreteSgmPath : public SgmPath
{
public:
    void init()
    {
        currentIndex = 0;
        currentPixel.x = startPixel.x;
        currentPixel.y = startPixel.y;
        previousPixel.X = currentPixel.x;
        previousPixel.y = currentPixel.y;


    }

};

// Start at { y,  0}
// Move by  { 0, +1}
// Finish   { y,C-1}
class Path_Str_XPos : public SgmPath
{
public:
    void init()
    {
        currentIndex = 0;
        length = std::min(length, imageWidth - startPixel.x);

        currentPixel.x = startPixel.x;
        currentPixel.y = startPixel.y;
        previousPixel.x = currentPixel.x;
        previousPixel.y = currentPixel.y;
    }

    void next()
    {
        ++currentIndex;
        previousPixel.x = currentPixel.x;
        currentPixel.x = currentPixel.x + 1;
    }

    static Point2 GetBorderPixel(Point2 pixel, int rows, int cols)
    {
        return Point2{pixel.y, 0};
    }
};

// Start at { y,C-1}
// Move by  { 0, -1}
// Finish   { y,  0}
class Path_Str_XNeg : public SgmPath
{
public:
    void init()
    {
        currentIndex = 0;
        length = std::min(length, startPixel.x + 1);

        currentPixel.x = startPixel.x;
        currentPixel.y = startPixel.y;
        previousPixel.x = currentPixel.x;
        previousPixel.y = currentPixel.y;
    }

    void next()
    {
        ++currentIndex;
        previousPixel.x = currentPixel.x;
        currentPixel.x = currentPixel.x - 1;
    }

    static Point2 GetBorderPixel(Point2 pixel, int rows, int cols)
    {
        return Point2{pixel.y, cols - 1};
    }
};

class Path_Str_YPos : public SgmPath
{
public:
    void init()
    {
        currentIndex = 0;
        length = std::min(length, imageHeight - startPixel.y);

        currentPixel.x = startPixel.x;
        currentPixel.y = startPixel.y;
        previousPixel.x = currentPixel.x;
        previousPixel.y = currentPixel.y;
    }

    void next()
    {
        ++currentIndex;
        previousPixel.y = currentPixel.y;
        currentPixel.y = currentPixel.y + 1;
    }

    static Point2 GetBorderPixel(Point2 pixel, int rows, int cols)
    {
        return Point2{0, pixel.x};
    }
};

class Path_Str_YNeg : public SgmPath
{
public:
    void init()
    {
        currentIndex = 0;
        length = std::min(length, startPixel.y + 1);

        currentPixel.x = startPixel.x;
        currentPixel.y = startPixel.y;
        previousPixel.x = currentPixel.x;
        previousPixel.y = currentPixel.y;
    }

    void next()
    {
        ++currentIndex;
        previousPixel.y = currentPixel.y;
        currentPixel.y = currentPixel.y - 1;
    }

    static Point2 GetBorderPixel(Point2 pixel, int rows, int cols)
    {
        return Point2{rows - 1, pixel.x};
    }
};
}
