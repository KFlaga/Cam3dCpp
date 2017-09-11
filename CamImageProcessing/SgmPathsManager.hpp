#pragma once

#include <CamCommon/Vector2.hpp>
#include <CamCommon/Array2d.hpp>
#include <CamCommon/Array3d.hpp>
#include <CamImageProcessing/SgmPath.hpp>
#include <functional>

namespace cam3d
{

class SgmPathsManager
{
public:
    struct DisparityCost
    {
        double cost;
        int disparity;
    };

    static constexpr int rows = 10;
    static constexpr int cols = 10;
    static constexpr int pathsCount = 8;

    Array3d<SgmPath*, rows, cols, pathsCount> paths;
    Array3d<DisparityCost, rows, cols, pathsCount> bestPathsCost;
    SgmPath::BorderPixelGetter borderPixelGetters[pathsCount];
    std::function<double(Point2, Point2)> getCost;

    int pathsInRun_RightTopDown[] = {
        PathDirection::PosX,
        PathDirection::PosY,
        PathDirection::PosX_PosY,
        PathDirection::NegX_PosY
    };

    int pathsInRun_RightBottomUp[] = {
        PathDirection::NegX,
        PathDirection::NegY,
        PathDirection::PosX_NegY,
        PathDirection::NegX_NegY
    };

    int pathsInRun_LeftTopDown[] = {
        PathDirection::NegX,
        PathDirection::PosY,
        PathDirection::PosX_PosY,
        PathDirection::NegX_PosY
    };

    int pathsInRun_LeftBottomUp[] = {
        PathDirection::PosX,
        PathDirection::NegY,
        PathDirection::PosX_NegY,
        PathDirection::NegX_NegY
    };

    void init(std::function<double(Point2, Point2)> costGetter)
    {
        getCost = costGetter;
        createBorderPaths();
        initBorderPixelGetters();
    }

    // private:
    int getDispRange(Point2 pixel)
    {
        return isLeftImageBase ?
                    pixel.x :
                    cols - 1 - pixel.x;
    }

    void createBorderPaths()
    {
        paths.fill(nullptr);
        for(int x = 0; x < cols; ++x)
        {
            createPathsForBorderPixel({0, x});
            initZeroStep({0, x});

            createPathsForBorderPixel({rows - 1, x});
            initZeroStep({rows - 1, x});
        }

        for(int y = 1; y < rows; ++y)
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
            paths(pixel, PathDirection::PosX) = new Path<PathDirection::PosX>{};
        }
        if(pixel.x == cols - 1)
        {
            paths(pixel, PathDirection::NegX) = new Path<PathDirection::NegX>{};
        }
        if(pixel.y == 0)
        {
            paths(pixel, PathDirection::PosY) = new Path<PathDirection::PosY>{};
        }
        if(pixel.y == rows - 1)
        {
            paths(pixel, PathDirection::NegY) = new Path<PathDirection::NegY>{};
        }
        if(pixel.x == 0 || pixel.y == 0)
        {
            paths(pixel, PathDirection::PosX_PosY) = new Path<PathDirection::PosX_PosY>{};
        }
        if(pixel.x == cols - 1 || pixel.y == 0)
        {
            paths(pixel, PathDirection::NegX_PosY) = new Path<PathDirection::NegX_PosY>{};
        }
        if(pixel.x == 0 || pixel.y == rows - 1)
        {
            paths(pixel, PathDirection::PosX_NegY) = new Path<PathDirection::PosX_NegY>{};
        }
        if(pixel.x == cols - 1 || pixel.y == rows - 1)
        {
            paths(pixel, PathDirection::NegX_NegY) = new Path<PathDirection::NegX_NegY>{};
        }
    }

    void initZeroStep(Point2 borderPixel)
    {
        for(int i = 0; i < pathsCount; ++i)
        {
            SgmPath* path = paths(borderPixel, i);
            if(path != nullptr)
            {
                path->imageHeight = rows;
                path->imageWidth = cols;
                path->startPixel = borderPixel;
                path->length = rows + cols;
                path->lastStepCosts = new double[cols];
                path->init();

                findInitialCostOnPath(borderPixel, path, i);
            }
        }
    }

    void findInitialCostOnPath(Point2 borderPixel, SgmPath* path, int pathNum)
    {
        int bestDisp = 0;
        double bestCost = costComp.getMaxCost() + 1.0;
        matched.y = borderPixel.y;

        // If base is right, then for each base pixel, matched one is on the right - disparity is positive
        int maxDisp = getDispRange(path->currentPixel);
        for(int d = 0; d < maxDisp; ++d)
        {
            matched.x = isLeftImageBase ? path->currentPixel.x - d : path->currentPixel.x + d;
            double cost = getCost(borderPixel, matched);
            path->lastStepCosts[d] = cost;

            if(bestCost > cost)
            {
                bestCost = cost;
                bestDisp = d;
            }
        }
        bestPathsCosts(path->currentPixel, pathNum) = DisparityCost{bestCost, bestDisp};
    }

    void initBorderPixelGetters()
    {
        borderPixelGetters[PathDirection::PosX] = &Path<PathDirection::PosX>::getBorderPixel;
        borderPixelGetters[PathDirection::NegX] = &Path<PathDirection::NegX>::getBorderPixel;
        borderPixelGetters[PathDirection::PosY] = &Path<PathDirection::PosY>::getBorderPixel;
        borderPixelGetters[PathDirection::NegY] = &Path<PathDirection::NegY>::getBorderPixel;
        borderPixelGetters[PathDirection::PosX_PosY] = &Path<PathDirection::PosX_PosY>::getBorderPixel;
        borderPixelGetters[PathDirection::NegX_PosY] = &Path<PathDirection::NegX_PosY>::getBorderPixel;
        borderPixelGetters[PathDirection::PosX_NegY] = &Path<PathDirection::PosX_NegY>::getBorderPixel;
        borderPixelGetters[PathDirection::NegX_NegY] = &Path<PathDirection::NegX_NegY>::getBorderPixel;
    }
};
}
