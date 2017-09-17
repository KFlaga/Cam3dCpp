#pragma once

#include <CamCommon/Vector2.hpp>

namespace cam3d
{
namespace PathDirections
{
enum PathDirection_ : int
{
    PosX, NegX, PosY, NegY,
    PosX_PosY, NegX_PosY, PosX_NegY, NegX_NegY,
    PosX2_PosY, NegX2_PosY, PosX2_NegY, NegX2_NegY,
    PosX_PosY2, NegX_PosY2, PosX_NegY2, NegX_NegY2,
};
}
using PathDirection = PathDirections::PathDirection_;


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

    using BorderPixelGetter = Point2(*)(Point2, int, int);
};

template<int moveY, int moveX>
class ConcreteSgmPath : public SgmPath
{
public:
    void init()
    {
        currentIndex = 0;
        currentPixel = startPixel;
        previousPixel = currentPixel;
        length = length_helper::getLength(length, imageHeight, imageWidth, startPixel);
    }

    void next()
    {
        previousPixel = currentPixel;
        currentPixel = currentPixel + move_helper::getMove(currentIndex);
        ++currentIndex;
    }

    static Point2 getBorderPixel(Point2 pixel, int rows, int cols)
    {
        return border_helper::getPixel(pixel, rows, cols);
    }
private:
	template<typename T>
    static constexpr T getAbs(const T i)
    {
        return i < 0 ? -i : i;
    }

    struct length_helper
    {
    public:
        static int getLength(int length, int rows, int cols, Point2 startPixel)
        {
            return std::min(
                    getLength2<0, moveX>(length, rows, cols, startPixel),
                    getLength2<moveY, 0>(length, rows, cols, startPixel)
                );
        }

        template<int y, int x>
        static typename std::enable_if<(x == 0)&&(y == 0), int>::type getLength2(
                int length, int rows, int cols, Point2 startPixel)
        {
            return length;
        }

        template<int y, int x>
        static typename std::enable_if<(x > 0)&&(y == 0), int>::type getLength2(
                int length, int rows, int cols, Point2 startPixel)
        {
            return (cols - startPixel.x) * x; // on 2 -> (c - x) * 2
        }

        template<int y, int x>
        static typename std::enable_if<(x < 0)&&(y == 0), int>::type getLength2(
                int length, int rows, int cols, Point2 startPixel)
        {
            return (startPixel.x + 1) * (-x); // on 2 -> x * 2 + 1
        }

        template<int y, int x>
        static typename std::enable_if<(x == 0)&&(y > 0), int>::type getLength2(
                int length, int rows, int cols, Point2 startPixel)
        {
            return (rows - startPixel.y) * y;
        }

        template<int y, int x>
        static typename std::enable_if<(x == 0)&&(y < 0), int>::type getLength2(
                int length, int rows, int cols, Point2 startPixel)
        {
            return (startPixel.y + 1) * (-y);
        }
    };

    struct move_helper
    {
        static Point2 getMove(int currentIndex)
        {
            return Point2{ 
				std::conditional<getAbs(moveY) == 2, Move2<moveY>, Move01<moveY>>::type::getMove(currentIndex),
				std::conditional<getAbs(moveX) == 2, Move2<moveX>, Move01<moveX>>::type::getMove(currentIndex)
			};
        }

		template<int m>
		struct Move2
		{
			static int getMove(int currentIndex)
			{
				static const int m2 = m / 2;
				return (currentIndex & 1) == 0 ? 0 : m2;
			}
		};

		template<int m>
		struct Move01
		{
			static int getMove(int currentIndex)
			{
				return m;
			}
		};
    };

    struct border_helper
    {
        static Point2 getPixel(Point2 pixel, int rows, int cols)
        {
			return std::conditional<(getAbs(moveX) < 2) && (getAbs(moveY) < 2), Pixel01, Pixel2
				>::type::getPixel(pixel, rows, cols);
        }

		struct Pixel01
		{
			static Point2 getPixel(Point2 pixel, int rows, int cols)
			{
				int d = Pixel01::getMoveLimit(pixel, rows, cols);
				return Point2{ pixel.y + d * (-moveY), pixel.x + d * (-moveX) };
			}

			static int getMoveLimit(Point2 pixel, int rows, int cols)
			{
				return std::min(
					getMoveLimit1<moveY, 0>(pixel, rows, cols),
					getMoveLimit1<0, moveX>(pixel, rows, cols)
				);
			}
		};

		struct Pixel2
		{
			static Point2 getPixel(Point2 pixel, int rows, int cols)
			{
				return std::conditional<getAbs(moveX) == 2, X2, Y2>::type::getPixel(pixel, rows, cols);
			}

			struct X2
			{
				static Point2 getPixel(Point2 pixel, int rows, int cols)
				{
					return{ 0, 0 };
				}
			};

			struct Y2
			{
				static Point2 getPixel(Point2 pixel, int rows, int cols)
				{
					// Move xy,x,xy,x
					// check if is bounded by x (compare lengths of opposite move)
					if (length_helper::getLength2(10000, rows, cols, pixel) >=
						length_helper::getLength2(10000, rows, cols, pixel))
					{
						int d = getMoveLimit1<y / 2, x / 2>(pixel, rows, cols);
						return Point2{ pixel.y + d * (-y) / 4, pixel.x + d * (-x) };
					}
				}
			};
		};

        template<int y, int x>
        static typename std::enable_if<(x == 0)&&(y == 0), int>::type getMoveLimit1(Point2 pixel, int rows, int cols)
        {
            return 10000;
        }

        template<int y, int x>
        static typename std::enable_if<(x == 1)&&(y == 0), int>::type getMoveLimit1(Point2 pixel, int rows, int cols)
        {
            return pixel.x;
        }

        template<int y, int x>
        static typename std::enable_if<(x == -1)&&(y == 0), int>::type getMoveLimit1(Point2 pixel, int rows, int cols)
        {
            return cols - pixel.x - 1;
        }

        template<int y, int x>
        static typename std::enable_if<(x == 0)&&(y == 1), int>::type getMoveLimit1(Point2 pixel, int rows, int cols)
        {
            return pixel.y;
        }

        template<int y, int x>
        static typename std::enable_if<(x == 0)&&(y == -1), int>::type getMoveLimit1(Point2 pixel, int rows, int cols)
        {
            return rows - pixel.y - 1;
        }
    };
};

template<PathDirection>
struct Path
{
};

template<> struct Path<PathDirection::PosX> : public ConcreteSgmPath<0, 1> { };
template<> struct Path<PathDirection::NegX> : public ConcreteSgmPath<0, -1> { };
template<> struct Path<PathDirection::PosY> : public ConcreteSgmPath<1, 0> { };
template<> struct Path<PathDirection::NegY> : public ConcreteSgmPath<-1, 0> { };
template<> struct Path<PathDirection::PosX_PosY> : public ConcreteSgmPath<1, 1> { };
template<> struct Path<PathDirection::NegX_PosY> : public ConcreteSgmPath<1, -1> { };
template<> struct Path<PathDirection::PosX_NegY> : public ConcreteSgmPath<-1, 1> { };
template<> struct Path<PathDirection::NegX_NegY> : public ConcreteSgmPath<-1, -1> { };
}
