#pragma once

#include <CamCommon/Vector2.hpp>
#include <CamCommon/Matrix.hpp>
#include <CamImageProcessing/CensusCostComputer.hpp>
#include <CamImageProcessing/GreyScaleImage.hpp>

namespace cam3d
{

using Disparity = double;
template<int rows, int cols>
using DisparityMap = Matrix<Disparity, rows, cols>;

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
    int currentIndex;
    Point2 previousPixel;
    int previousIndex;

    bool haveNextPixel()
    {
        return currentIndex < length - 1;
    }

    double* lastStepCosts; // Needs to be allocated externally

    virtual void init();
    virtual void next();

    using BorderPixelGetter = Point2(Point2, int, int);
};

class SgmDisparityComputer;

class SgmCostAggregator
{
public:
    struct DisparityCost
    {
        double cost;
        int disparity;
    };

    double lowPenaltyCoeff; // P1 = coeff * MaxCost
    double highPenaltyCoeff; // P2 = coeff * MaxCost * (1 - grad * |Ib - Im|)
    double gradientCoeff;
    int maxDisparity;
    int minDisparity;
    int dispRange;

    SgmDisparityComputer* disparityComputer;

private:
    double P1;
    double P2;

    static constexpr int rows = 10;
    static constexpr int cols = 10;
    static constexpr int pathsCount = 16;
    SgmPath* paths[rows][cols][pathsCount]; // TODO: array3d
    DisparityCost bestPathsCost[rows][cols][pathsCount];
    SgmPath::BorderPixelGetter borderPixelGetters[pathsCount];

    int pathsIntRun_RightTopDown[8];
    int pathsIntRun_RightBottomUp[8];
    int pathsIntRun_LeftTopDown[8];
    int pathsIntRun_LeftBottomUp[8];

    CensusCostComputer<GreyScaleImage<rows, cols>, BitWord32<8>> costComp;

    Point2 _matched;

    void createBorderPaths()
    {
        for(int y = 0; x < rows; ++y)
        {
            for(int x = 0; x < cols; ++x)
            {
                for(int i = 0; i < pathsCount; ++i)
                {
                    paths[y][x][i] = nullptr;
                }
            }
        }

        for(int x = 0; x < ImageBase.ColumnCount; ++x)
        {
            createPathsForBorderPixel({0, x});
            initZeroStep({0, x});

            createPathsForBorderPixel({rows - 1, x});
            initZeroStep({rows - 1, x});
        }

        for(int y = 1; y < ImageBase.RowCount; ++y)
        {
            createPathsForBorderPixel({y, 0});
            initZeroStep({y, 0});

            createPathsForBorderPixel({y, cols - 1});
            initZeroStep({y, cols - 1});
        }
    }

    void createPathsForBorderPixel(Point2 pixel)
    {
        // Create only those paths which can start on pixel (y,x)
        if(pixel.x == 0)
        {
            paths[pixel.y][pixel.x][PathDirection::PosX] = new Path_Str_XPos();
        }
        else if(pixel.x == cols - 1)
        {
            paths[pixel.y][pixel.x][PathDirection::NegX] = new Path_Str_XNeg();
        }

        if(pixel.y == 0)
        {
            paths[pixel.y][pixel.x][PathDirection::PosY] = new Path_Str_YPos();
        }
        else if(pixel.y == rows - 1)
        {
            paths[pixel.y][pixel.x][PathDirection::NegY] = new Path_Str_YNeg();
        }

        if(pixel.x == 0 || pixel.y == 0)
        {
            paths[pixel.y][pixel.x][PathDirection::PosX_PosY] = new Path_Diag_XPosYPos();
            paths[pixel.y][pixel.x][PathDirection::PosX2_PosY] = new Path_Diag2_X2PosYPos();
            paths[pixel.y][pixel.x][PathDirection::PosX_PosY2] = new Path_Diag2_XPosY2Pos();
        }
        if(pixel.x == cols - 1 || pixel.y == 0)
        {
            paths[pixel.y][pixel.x][PathDirection::NegX_PosY] = new Path_Diag_XNegYPos();
            paths[pixel.y][pixel.x][PathDirection::NegX2_PosY] = new Path_Diag2_X2NegYPos();
            paths[pixel.y][pixel.x][PathDirection::NegX_PosY2] = new Path_Diag2_XNegY2Pos();
        }
        if(pixel.x == 0 || pixel.y == rows - 1)
        {
            paths[pixel.y][pixel.x][PathDirection::PosX_NegY] = new Path_Diag_XPosYNeg();
            paths[pixel.y][pixel.x][PathDirection::PosX2_NegY] = new Path_Diag2_X2PosYNeg();
            paths[pixel.y][pixel.x][PathDirection::PosX_NegY2] = new Path_Diag2_XPosY2Neg();
        }
        if(pixel.x == cols - 1 || pixel.y == rows - 1)
        {
            paths[pixel.y][pixel.x][PathDirection::NegX_NegY] = new Path_Diag_XNegYNeg();
            paths[pixel.y][pixel.x][PathDirection::NegX2_NegY] = new Path_Diag2_X2NegYNeg();
            paths[pixel.y][pixel.x][PathDirection::NegX_NegY2] = new Path_Diag2_XNegY2Neg();
        }
    }

    void initZeroStep(Point2 borderPixel)
    {
        for(int i = 0; i < pathsCount; ++i)
        {
            SgmPath* path = paths[borderPixel.y][borderPixel.x][i];
            if(path != nullptr)
            {
                path->imageHeight = rows;
                path->imageWidth = cols;
                path->startPixel = borderPixel;
                path->length = rows + cols;
                path->lastStepCosts = new double[dispRange + 1];
                path->init();

                int bestDisp = 0;
                double bestCost = costComp.getMaxCost() + 1.0;
                _matched.y = borderPixel.y;

                // If base is right, then for each base pixel, matched one is on the right - disparity is positive
                int maxDisp = IsLeftImageBase ? path->currentPixel.x : cols - 1 - path->currentPixel.x;

                for(int d = 0; d < maxDisp; ++d)
                {
                    _matched.x = IsLeftImageBase ? path->currentPixel.x - d : path->currentPixel.x + d;
                    double cost = costComp.getCostOnBorder(borderPixel, _matched);
                    path->lastStepCosts[d] = cost;

                    if(bestCost > cost)
                    {
                        bestCost = cost;
                        bestDisp = d;
                    }
                }
                _bestPathCosts[path->currentPixel.y][path->currentPixel.x][i] = DisparityCost{bestCost, bestDisp};
            }
        }
    }


};
}
