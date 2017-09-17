#pragma once

#include <CamCommon/Vector2.hpp>
#include <CamCommon/Array2d.hpp>
#include <CamCommon/Array3d.hpp>
#include <CamImageProcessing/SgmPath.hpp>
#include <functional>
#include <array>

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

    enum class RunDirection
    {
        TopDown,
        BottomUp
    };

    static constexpr int pathsCount = 8;

private:
	int rows;
	int cols;
    Array3d<SgmPath*> paths;
    Array3d<DisparityCost> bestPathsCosts;
    SgmPath::BorderPixelGetter borderPixelGetters[pathsCount];
    std::function<double(Point2, Point2)> getCost;
    bool isLeftImageBase;

    static constexpr int pathsPerRun = pathsCount / 2;
    std::array<int, pathsPerRun> pathsInRun_RightTopDown = {
        PathDirection::PosX,
        PathDirection::PosY,
        PathDirection::PosX_PosY,
        PathDirection::NegX_PosY
    };

    std::array<int, pathsPerRun> pathsInRun_RightBottomUp = {
        PathDirection::NegX,
        PathDirection::NegY,
        PathDirection::PosX_NegY,
        PathDirection::NegX_NegY
    };

    std::array<int, pathsPerRun> pathsInRun_LeftTopDown = {
        PathDirection::NegX,
        PathDirection::PosY,
        PathDirection::PosX_PosY,
        PathDirection::NegX_PosY
    };

    std::array<int, pathsPerRun> pathsInRun_LeftBottomUp = {
        PathDirection::PosX,
        PathDirection::NegY,
        PathDirection::PosX_NegY,
        PathDirection::NegX_NegY
    };

public:
    SgmPathsManager(int rows_, int cols_, std::function<double(Point2, Point2)> getCost_, bool isLeftImageBase_) :
		rows{ rows_ },
		cols{ cols_ },
		getCost{ getCost_ },
		isLeftImageBase(isLeftImageBase_),
		paths{rows, cols, pathsCount},
		bestPathsCosts{rows, cols, pathsCount}
    {
        createBorderPaths();
        initBorderPixelGetters();
    }

    std::array<int, pathsPerRun> getPathForRun(RunDirection dir)
    {
        return dir == RunDirection::TopDown ?
            (isLeftImageBase ? pathsInRun_LeftTopDown : pathsInRun_RightTopDown) :
            (isLeftImageBase ? pathsInRun_LeftBottomUp : pathsInRun_RightBottomUp);
    }

    int getDispRange(Point2 pixel)
    {
        return isLeftImageBase ? pixel.x : cols - 1 - pixel.x;
    }

    SgmPath* getPath(Point2 pixel, int pathNum) const
    {
        return paths(pixel, pathNum);
    }

    DisparityCost getBestPathCosts(Point2 pixel, int pathNum) const
    {
        return bestPathsCosts(pixel, pathNum);
    }

    void setBestPathCosts(Point2 pixel, int pathNum, DisparityCost cost)
    {
        bestPathsCosts(pixel, pathNum) = cost;
    }

    Point2 getBorderPixel(Point2 point, int pathDir)
    {
        return borderPixelGetters[pathDir](point, rows, cols);
    }

private:
    void createBorderPaths()
    {
        paths.fill(0);
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
                path->lastStepCosts = new double[cols + 1]; // Need cols + 1 for convinent bottom-up run
                path->init();

                findInitialCostOnPath(path, i);
            }
        }
    }

    void findInitialCostOnPath(SgmPath* path, int pathNum)
    {
        int bestDisp = 0;
        double bestCost = 1e12;
        int maxDisp = getDispRange(path->currentPixel);
        for(int d = 0; d < maxDisp; ++d)
        {
            double cost = getCost(path->currentPixel,
                                  path->currentPixel + Point2{ 0, isLeftImageBase ? -d : d });
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
