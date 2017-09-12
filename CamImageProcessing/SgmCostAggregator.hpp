#pragma once

#include <CamCommon/Vector2.hpp>
#include <CamCommon/Array2d.hpp>
#include <CamCommon/Array3d.hpp>
#include <CamCommon/BitWord.hpp>
#include <CamImageProcessing/CensusCostComputer.hpp>
#include <CamImageProcessing/SgmPathsManager.hpp>
#include <stdexcept>

namespace cam3d
{

using Disparity = double;
template<int rows, int cols>
using DisparityMap = Array2d<Disparity, rows, cols>;

class SgmDisparityComputer;

template<typename Image_, int rows = Image_::Matrix::rows, int cols = Image_::Matrix::cols>
class SgmCostAggregator
{
public:
    using Image = Image_;
    static constexpr int pathsCount = 8;
    static constexpr int isLeftImageBase = true;

    double lowPenaltyCoeff; // P1 = coeff * MaxCost
    double highPenaltyCoeff; // P2 = coeff * MaxCost * (1 - grad * |Ib - Im|)
    double gradientCoeff;

public:
    using PathsManager = SgmPathsManager<rows, cols>;
    using DisparityCost = typename PathsManager::DisparityCost;
    using RunDirection = typename PathsManager::RunDirection;
    PathsManager pathMgr;
    SgmDisparityComputer* disparityComputer;

public:
    double P1;
    double P2;

    using Bitword = BitWord32<8>;
    CensusCostComputer<Image, Bitword> costComp;

    Point2 currentPixel;
    Point2 matched;

    void computeMatchingCosts(Image& baseImage, Image& matchedImage)
    {
        costComp.init(baseImage, matchedImage);
        P1 = lowPenaltyCoeff * costComp.getMaxCost();
        P2 = highPenaltyCoeff * costComp.getMaxCost();

        pathMgr.init([this](Point2 p1, Point2 p2){ return this->getCost(p1, p2); }, isLeftImageBase);

        findCostsTopDown();
        findCostsBottomUp();
        findDisparities();
    }

    int getDispRange(Point2 pixel)
    {
        return isLeftImageBase ?
                    pixel.x :
                    cols - 1 - pixel.x;
    }

    void findCostsTopDown()
    {
        for(int y = 0; y < rows; ++y)
        {
            for(int x = 0; x < cols; ++x)
            {
                findCostsForPixel(y, x, RunDirection::TopDown);
            }
        }
    }

    void findCostsBottomUp()
    {
        for(int y = rows - 1; y >= 0; --y)
        {
            for(int x = cols - 1; x >= 0; --x)
            {
                findCostsForPixel(y, x, RunDirection::BottomUp);
            }
        }
    }

    void findCostsForPixel(int y, int x, RunDirection dir)
    {
        currentPixel = {y, x};
        matched = currentPixel;

        for(int pathIdx : pathMgr.getPathForRun(dir))
        {
            findCostsForPath(pathIdx, dir == RunDirection::BottomUp);
        }
    }

    void findCostsForPath(int pathIdx, bool isBottomUp)
    {
        Point2 borderPixel = pathMgr.getBorderPixel(currentPixel, pathIdx);
        SgmPath* path = pathMgr.getPath(borderPixel, pathIdx);
        checkPathCorrectness(path);

        // If base is right, then for each base pixel, matched one is on the right - disparity is positive
        int maxDisp = getDispRange(path->currentPixel);
        int bestDisp = 0;
        double bestCost = 1e12;
        DisparityCost bestPrev = pathMgr.getBestPathCosts(path->previousPixel, pathIdx);
        double prevCost = bestCost;

        for(int d = 0; d < maxDisp; ++d)
        {
            matched.x = isLeftImageBase ? path->currentPixel.x - d : path->currentPixel.x + d;

            double cost = findCost(path->currentPixel, path, d, bestPrev.disparity, bestPrev.cost, maxDisp, prevCost);
            prevCost = path->lastStepCosts[d];
            path->lastStepCosts[d] = cost;

            // Save best disparity at current path index
            if(bestCost > cost)
            {
                bestCost = cost;
                bestDisp = d;
            }
        }
        pathMgr.setBestPathCosts(path->currentPixel, pathIdx, {bestCost, bestDisp});

        if(isBottomUp && maxDisp > 0)
        {
            alignForBottomUp(path, maxDisp);
        }

        path->next();
    }

    void alignForBottomUp(SgmPath* path, int maxDisp)
    {
        // For disparity greater than max, matched pixel will exceed image dimensions:
        // L[p, d > dmax-1] = Cost(curPix, maxXPix) + LastCost[dmax-1]
        // We actualy need only to compute L[p, dmax] as it will be needed in next iteration
        matched.x = isLeftImageBase ? 0 : cols - 1;
        path->lastStepCosts[maxDisp] = // As LastStepCosts is of size dispRange + 1 we won't exceed max index
                getCost(currentPixel, matched) + path->lastStepCosts[maxDisp - 1];
    }

    void checkPathCorrectness(SgmPath* path)
    {
        if(path == nullptr)
        {
            throw std::logic_error(std::string("File: ") + __FILE__ + ", line: " + std::to_string(__LINE__));
        }
        if(path->length <= 0)
        {
            throw std::logic_error(std::string("File: ") + __FILE__ + ", line: " + std::to_string(__LINE__));
            //bestPathsCosts[path.CurrentPixel.Y, path.CurrentPixel.X, pathIdx] = new DisparityCost(0, 1e12);
        }
        if(path->currentIndex >= path->length)
        {
            throw std::logic_error(std::string("File: ") + __FILE__ + ", line: " + std::to_string(__LINE__));
            //bestPathsCosts[path.CurrentPixel.Y, path.CurrentPixel.X, pathIdx] = new DisparityCost(0, 1e12);
        }
    }

    void findDisparities()
    {
        // 3rd run: compute final disparity based on paths' bests
        // Set disparity to be weighted average of best disparities
        for(int r = 0; r < rows; ++r)
        {
            for(int c = 0; c < cols; ++c)
            {
                currentPixel = {r, c};
                matched = currentPixel;

                for(int i = 0; i < PathsManager::pathsCount; ++i)
                {
                    matched.x = currentPixel.x + pathMgr.getBestPathCosts(currentPixel, i).disparity;
                    double matchCost = getCost(currentPixel, matched);
//                    DispComp.StoreDisparity(new Disparity()
//                    {
//                        DX = IsLeftImageBase ? -bestPathsCosts[CurrentPixel.Y, CurrentPixel.X, i].Disparity :
//                          bestPathsCosts[CurrentPixel.Y, CurrentPixel.X, i].Disparity,
//                        DY = 0,
//                        Cost = matchCost
//                    });
                }
//                DispComp.FinalizeForPixel(CurrentPixel);
            }
        }
    }

    double findCost(Point2 basePixel, SgmPath* path, int d, int bestPrevDisp, double bestPrevCost, int dmax, double prevCost)
    {
        double pen0, pen1, pen2;

        pen0 = path->lastStepCosts[d];
        if(d == 0)
            pen1 = path->lastStepCosts[d + 1];
        else if(d > dmax - 2) // orginal : if(d > _dispRange - 2)
            pen1 = prevCost; // path.LastStepCosts[d - 1];
        else
            pen1 = std::min(path->lastStepCosts[d + 1], prevCost);//  path.LastStepCosts[d - 1]);

        pen2 = 1e12;
        if(bestPrevDisp < d - 1 || bestPrevDisp > d + 1)
        {
            pen2 = bestPrevCost;
        }
        else
        {
            for(int dk = 0; dk < d - 1; ++dk)
            {
                pen2 = std::min(path->lastStepCosts[dk], pen2);
            }

            for(int dk = d + 2; dk < dmax - 1; ++dk)
            {
                pen2 = std::min(path->lastStepCosts[dk], pen2);
            }
        }

//        if(basePixel.x >= cols || matched.x >= cols ||
//            basePixel.x < 0 || matched.x < 0)
//        {
//            // ??
//        }

//        if(minFrom(pen0, pen1 + P1, pen2 + P2) // * (1.0 - gradientCoeff * std::abs(ImageBase[basePixel.Y, basePixel.X] - ImageMatched[matched.Y, matched.X])))) !=
//           !=
//           minFrom(pen0, pen1 + P1, bestPrevCost + P2)) // * (1.0 - GradientCoeff * std::abs(ImageBase[basePixel.Y, basePixel.X] - ImageMatched[matched.Y, matched.X])))))
//        {

//        }

        double c = costComp.getCost(basePixel, matched);
        return c + minFrom(pen0, pen1 + P1, bestPrevCost + P2);// * (1.0 - GradientCoeff *
            //std::abs(ImageBase[basePixel.Y, basePixel.X] - ImageMatched[matched.Y, matched.X]))));
    }

    double getCost(Point2 basePixel, Point2 matchedPixel)
    {
        matchedPixel.x = std::max(0, std::min(matchedPixel.x, cols));
        matchedPixel.y = std::max(0, std::min(matchedPixel.y, rows));

        return costComp.getCost(basePixel, matchedPixel);
    }
};
}
