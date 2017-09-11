#pragma once

#include <CamCommon/Vector2.hpp>
#include <CamCommon/Array2d.hpp>
#include <CamCommon/Array3d.hpp>
#include <CamImageProcessing/CensusCostComputer.hpp>
#include <CamImageProcessing/GreyScaleImage.hpp>
#include <CamImageProcessing/SgmPathsManager.hpp>

namespace cam3d
{

using Disparity = double;
template<int rows, int cols>
using DisparityMap = Array2d<Disparity, rows, cols>;

class SgmDisparityComputer;

class SgmCostAggregator
{
public:
    bool isLeftImageBase;
    double lowPenaltyCoeff; // P1 = coeff * MaxCost
    double highPenaltyCoeff; // P2 = coeff * MaxCost * (1 - grad * |Ib - Im|)
    double gradientCoeff;

    SgmDisparityComputer* disparityComputer;
    SgmPathsManager paths;

public:
    double P1;
    double P2;

    static constexpr int rows = 10;
    static constexpr int cols = 10;
    static constexpr int pathsCount = 8;
    using Image = GreyScaleImage<rows, cols>;
    using Bitword = BitWord32<8>;

    CensusCostComputer<Image, Bitword> costComp;

    Point2 matched;

    void computeMatchingCosts()
    {
        P1 = lowPenaltyCoeff * costComp.getMaxCost();
        P2 = highPenaltyCoeff * costComp.getMaxCost();

        paths.init();

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

    double findCost(IntVector2 basePixel, Path path, int d, int bestPrevDisp, double bestPrevCost, int dmax, double prevCost)
    {
        double pen0, pen1, pen2;

        pen0 = path.LastStepCosts[d];
        if(d == 0)
            pen1 = path.LastStepCosts[d + 1];
        else if(d > _dispRange - 2)
            pen1 = prevCost; // path.LastStepCosts[d - 1];
        else
            pen1 = Math.Min(path.LastStepCosts[d + 1], prevCost);//  path.LastStepCosts[d - 1]);

        pen2 = double.MaxValue;
        if(bestPrevDisp < d - 1 || bestPrevDisp > d + 1)
        {
            pen2 = bestPrevCost;
        }
        else
        {
            for(int dk = 0; dk < d - 1; ++dk)
            {
                pen2 = Math.Min(path.LastStepCosts[dk], pen2);
            }

            for(int dk = d + 2; dk < dmax - 1; ++dk)
            {
                pen2 = Math.Min(path.LastStepCosts[dk], pen2);
            }
        }

        if(basePixel.X >= ImageBase.ColumnCount || matched.X >= ImageBase.ColumnCount ||
            basePixel.X < 0 || matched.X < 0)
        {

        }

        if(Math.Min(pen0, Math.Min(pen1 + _penaltyLow, pen2 + _penaltyHigh * (1.0 - GradientCoeff *
            Math.Abs(ImageBase[basePixel.Y, basePixel.X] - ImageMatched[matched.Y, matched.X])))) !=
            Math.Min(
            pen0, Math.Min(pen1 + _penaltyLow, bestPrevCost + _penaltyHigh * (1.0 - GradientCoeff *
            Math.Abs(ImageBase[basePixel.Y, basePixel.X] - ImageMatched[matched.Y, matched.X])))))
        {

        }

        //return CostComp.GetCost_Border(basePixel, _matched) + Math.Min(
        //    pen0, Math.Min(pen1 + _penaltyLow, pen2 + _penaltyHigh * (1.0 - GradientCoeff *
        //    Math.Abs(ImageBase[basePixel.Y, basePixel.X] - ImageMatched[_matched.Y, _matched.X]))));


        double c = CostComp.GetCost_Border(basePixel, matched);
        return c + Math.Min(
            pen0, Math.Min(pen1 + _penaltyLow, bestPrevCost + _penaltyHigh * (1.0 - GradientCoeff *
            Math.Abs(ImageBase[basePixel.Y, basePixel.X] - ImageMatched[matched.Y, matched.X]))));
    }

    double getCost(Point2 pixel, Disparity disp)
    {
        return getCost(pixel, disp.GetMatchedPixel(pixel));
    }

    double getCost(Point2 basePixel, Point2 matchedPixel)
    {
        matchedPixel.X = Math.Max(0, Math.Min(matchedPixel.X, ImageBase.ColumnCount - 1));
        matchedPixel.Y = Math.Max(0, Math.Min(matchedPixel.Y, ImageBase.RowCount - 1));

        return CostComp.GetCost_Border(basePixel, matchedPixel);
    }

    void findCostsTopDown()
    {
        int[] paths = IsLeftImageBase ? _pathsInRun_LeftTopDown : _pathsInRun_RightTopDown;
        // 1st run: start from (0,0) move left/downwards
        for(int r = 0; r < ImageBase.RowCount; ++r)
        {
            for(int c = 0; c < ImageBase.ColumnCount; ++c)
            {
                CurrentPixel.Set(x: c, y: r);
                matched.Y = CurrentPixel.Y;

                foreach(int pathIdx in paths)
                {
                    FindCostsForPath(pathIdx, false);
                }
            }
        }
    }

    void findCostsBottomUp()
    {
        int[] paths = IsLeftImageBase ? _pathsInRun_LeftBottomUp : _pathsInRun_RightBottomUp;
        // 2nd run: start from (rows,cols) move right/upwards
        for(int r = ImageBase.RowCount - 1; r >= 0; --r)
        {
            for(int c = ImageBase.ColumnCount - 1; c >= 0; --c)
            {
                CurrentPixel.Set(x: c, y: r);
                matched.Y = CurrentPixel.Y;

                foreach(int pathIdx in paths)
                {
                    FindCostsForPath(pathIdx, true);
                }
            }
        }
    }

    void findDisparities()
    {
        // 3rd run: compute final disparity based on paths' bests
        // Set disparity to be weighted average of best disparities
        for(int r = 0; r < ImageBase.RowCount; ++r)
        {
            for(int c = 0; c < ImageBase.ColumnCount; ++c)
            {
                CurrentPixel.Set(x: c, y: r);
                matched.Y = CurrentPixel.Y;

                for(int i = 0; i < 16; ++i)
                {
                    matched.X = CurrentPixel.X + bestPathsCosts[CurrentPixel.Y, CurrentPixel.X, i].Disparity;
                    double matchCost = GetCost(CurrentPixel, matched);
                    DispComp.StoreDisparity(new Disparity()
                    {
                        DX = IsLeftImageBase ? -bestPathsCosts[CurrentPixel.Y, CurrentPixel.X, i].Disparity :
                          bestPathsCosts[CurrentPixel.Y, CurrentPixel.X, i].Disparity,
                        DY = 0,
                        Cost = matchCost
                    });
                }
                DispComp.FinalizeForPixel(CurrentPixel);
            }
        }
    }


    void findCostsForPath(int pathIdx, bool startedOnZeroRange)
    {
        IntVector2 borderPixel = _borderPixelGetters[pathIdx](
                                        CurrentPixel, ImageBase.RowCount, ImageBase.ColumnCount);

        Path path = _paths[borderPixel.Y, borderPixel.X][pathIdx];
        if(path.Length <= 0)
        {
            bestPathsCosts[path.CurrentPixel.Y, path.CurrentPixel.X, pathIdx] = new DisparityCost(0, 1e12);
            return;
        }
        if(path.CurrentIndex >= path.Length)
        {
            //bestPathsCosts[path.CurrentPixel.Y, path.CurrentPixel.X, pathIdx] = new DisparityCost(0, 1e12);
            return;
        }

        // If base is right, then for each base pixel, matched one is on the right - disparity is positive
        int maxDisp = IsLeftImageBase ? path.CurrentPixel.X : ImageBase.ColumnCount - 1 - path.CurrentPixel.X;
        int bestDisp = 0;
        double bestCost = double.MaxValue;
        DisparityCost bestPrev = bestPathsCosts[path.PreviousPixel.Y, path.PreviousPixel.X, pathIdx];
        double prevCost = double.MaxValue;

        for(int d = 0; d < maxDisp; ++d)
        {
            matched.X = IsLeftImageBase ? path.CurrentPixel.X - d : path.CurrentPixel.X + d;

            double cost = FindCost_Rect(path.CurrentPixel, path, d, bestPrev.Disparity, bestPrev.Cost, maxDisp, prevCost);
            prevCost = path.LastStepCosts[d];
            path.LastStepCosts[d] = cost;

            // Save best disparity at current path index
            if(bestCost > cost)
            {
                bestCost = cost;
                bestDisp = d;
            }
        }
        bestPathsCosts[path.CurrentPixel.Y, path.CurrentPixel.X, pathIdx] = new DisparityCost(bestDisp, bestCost);

        if(startedOnZeroRange && maxDisp > 0)
        {
            // For disparity greater than max, matched pixel will exceed image dimensions:
            // L[p, d > dmax-1] = Cost(curPix, maxXPix) + LastCost[dmax-1]
            // We actualy need only to compute L[p, dmax] as it will be needed in next iteration
            matched.X = IsLeftImageBase ? 0 : ImageBase.ColumnCount - 1;
            path.LastStepCosts[maxDisp] = // As LastStepCosts is of size dispRange + 1 we won't exceed max index
                GetCost(CurrentPixel, matched) + path.LastStepCosts[maxDisp - 1];
        }

        path.Next();
    }
};
}
