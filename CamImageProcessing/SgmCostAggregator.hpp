#pragma once

#include <CamCommon/Vector2.hpp>
#include <CamCommon/Array2d.hpp>
#include <CamCommon/Array3d.hpp>
#include <CamCommon/BitWord.hpp>
#include <CamCommon/DisparityMap.hpp>
#include <CamImageProcessing/CensusCostComputer.hpp>
#include <CamImageProcessing/SgmPathsManager.hpp>
#include <CamImageProcessing/SgmDisparityComputer.hpp>
#include <stdexcept>
#include <atomic>

namespace cam3d
{
struct SgmParameters
{
    enum ImageType
    {
        Grey = 0,
        Masked = 1
    };

    int rows;
    int cols;
    ImageType imageType;

    double lowPenaltyCoeff;
    double highPenaltyCoeff;
    double gradientCoeff;
	// TODO: disparity range limits

    int censusMaskRadius;

    int disparityMeanMethod;
    int disparityCostMethod;
    double diparityPathLengthThreshold;
};

class ISgmCostAggregator
{
public:
	virtual void computeMatchingCosts() = 0;
	virtual void terminate() = 0;
	virtual std::string getState() = 0;
	virtual void setParameters(SgmParameters& params) = 0;
	virtual DisparityMap& getDisparityMap() = 0;
	virtual Point2 getCurrentPixel() = 0;
};

template<typename Image_, typename CostComputer_>
class SgmCostAggregator : public ISgmCostAggregator
{
public:
    using Image = Image_;
	using CostComputer = CostComputer_;
	using DisparityComputer = SgmDisparityComputer<Image, CostComputer>;
    static constexpr int pathsCount = 8;

private:
    int rows;
    int cols;
    double lowPenaltyCoeff; // P1 = coeff * MaxCost
    double highPenaltyCoeff; // P2 = coeff * MaxCost * (1 - grad * |Ib - Im|)
    double gradientCoeff;
    bool isLeftImageBase = true;

    using DisparityCost = typename SgmPathsManager::DisparityCost;
    using RunDirection = typename SgmPathsManager::RunDirection;
    SgmPathsManager pathMgr;

    double P1;
    double P2;

    CostComputer costComp;
    DisparityComputer dispComp;

    Point2 currentPixel;
    Point2 matched;

	Image& imageBase;
	Image& imageMatched;
	DisparityMap& map;

	std::atomic_bool shouldTerminate;
	std::string currentState;

public:
    SgmCostAggregator(int rows_, int cols_, bool isLeftImageBase_, Image& imageBase_, Image& imageMatched_, DisparityMap& map_) :
        rows{rows_},
        cols{cols_},
        isLeftImageBase{isLeftImageBase_},
		imageBase{imageBase_},
		imageMatched{imageMatched_},
		map{map_},
        pathMgr{rows, cols, [this](Point2 p1, Point2 p2){ return this->getCost(p1, p2); }, isLeftImageBase},
        costComp{rows, cols},
		dispComp{rows, cols, map, imageBase, imageMatched, costComp},
		currentState{"NotRun"}
    {

    }

    int getRows() const { return rows; }
    int getCols() const { return cols; }
	bool getIsLeftImageBase() const { return isLeftImageBase; }
	Image& getBaseImage() { return imageBase; }
	Image& getMatchedImage() { return imageMatched; }
	DisparityMap& getDisparityMap() override { return map; }
	Point2 getCurrentPixel() override { return currentPixel;  }

	void setParameters(SgmParameters& params) override;
	void terminate() override { shouldTerminate = true; }
	std::string getState() override { return currentState; }

    void computeMatchingCosts() override
    {
		shouldTerminate = false;
		currentState = "Init";
        costComp.init(imageBase, imageMatched);
        P1 = lowPenaltyCoeff * costComp.getMaxCost();
        P2 = highPenaltyCoeff * costComp.getMaxCost();

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
		currentState = "Run: TopDown";
        for(int y = 0; y < rows; ++y)
        {
            for(int x = 0; x < cols; ++x)
            {
				if (shouldTerminate)
					return;
                findCostsForPixel(y, x, RunDirection::TopDown);
            }
        }
    }

    void findCostsBottomUp()
    {
		currentState = "Run: BottomUp";
        for(int y = rows - 1; y >= 0; --y)
        {
            for(int x = cols - 1; x >= 0; --x)
            {
				if (shouldTerminate)
					return;
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
        int maxDisp = getDispRange(path->currentPixel);
        TEST_checkPathCorrectness(path);

        findCostForEachDisparityInStep(path, pathIdx, maxDisp);

        if(isBottomUp && maxDisp > 0)
        {
            alignForBottomUp(path, maxDisp);
        }

        path->next();
    }

    void findCostForEachDisparityInStep(SgmPath* path, int pathIdx, int maxDisp)
    {
        int bestDisp = 0;
        double bestCost = 1e12;
        auto bestPrev = pathMgr.getBestPathCosts(path->previousPixel, pathIdx);
        double prevCost = bestCost;

        for(int d = 0; d < maxDisp; ++d)
        {
            matched.x = isLeftImageBase ? path->currentPixel.x - d : path->currentPixel.x + d;

            double cost = findCostForDisparity(path->currentPixel, path, d, maxDisp, bestPrev.disparity, bestPrev.cost);
            prevCost = path->lastStepCosts[d];
            path->lastStepCosts[d] = cost;

            if(bestCost > cost)
            {
                bestCost = cost;
                bestDisp = d;
            }
        }
        pathMgr.setBestPathCosts(path->currentPixel, pathIdx, {bestCost, bestDisp});
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

    void TEST_checkPathCorrectness(SgmPath* path)
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

    double findCostForDisparity(Point2 basePixel, SgmPath* path, int d, int dmax, int bestPrevDisp, double bestPrevCost)
    {
        double pen0 = path->lastStepCosts[d];
        double pen1 = findPenaltyClose(path, d, dmax);
        double pen2 = findPenaltyFar(dmax, d, bestPrevCost, bestPrevDisp, path);
        TEST_checkPointsCorectness(basePixel, matched);
        TEST_checkCostSimplification(pen0, pen1, pen2, bestPrevCost);

        double c = costComp.getCost(basePixel, matched);
        return c + minFrom(pen0, pen1 + P1, pen2 + P2);
        // * (1.0 - gradientCoeff * std::abs(imageBase(basePixel.y, basePixel.x) - imageMatched(matched.y, matched.x)))));
    }

    double findPenaltyClose(SgmPath* path, int d, int dmax)
    {
        // TODO: profile and check if removing those 2 checks will speed it a bit
        //  it can be removed in 2 ways:
        //  1) make separate findCostForDisparity functions for d=0 and d=max-1
        //  2) enlarge lastStepCosts and move 1 left, so that we iterate from d=1 to d=max and
        //     on boundaries cost is max
        if(d == 0) { return path->lastStepCosts[d + 1]; }
        if(d > dmax - 2) /* orginal : if(d > _dispRange - 2) */ { return path->lastStepCosts[d - 1]; }
        return std::min(path->lastStepCosts[d + 1], path->lastStepCosts[d - 1]);
    }

    double findPenaltyFar(int dmax, int d, double bestPrevCost, int bestPrevDisp, SgmPath* path)
    {
        if(bestPrevDisp < d - 1 || bestPrevDisp > d + 1)
        {
            return bestPrevCost;
        }
        double pen2 = 1e12;
        for(int dk = 0; dk < d - 1; ++dk)
        {
            pen2 = std::min(path->lastStepCosts[dk], pen2);
        }
        for(int dk = d + 2; dk < dmax - 1; ++dk)
        {
            pen2 = std::min(path->lastStepCosts[dk], pen2);
        }
        return pen2;
    }

    void TEST_checkPointsCorectness(Point2 basePixel, Point2 matched)
    {
        if(basePixel.x >= cols || matched.x >= cols || basePixel.x < 0 || matched.x < 0)
        {
            throw std::logic_error(std::string("File: ") + __FILE__ + ", line: " + std::to_string(__LINE__));
        }
    }

    void TEST_checkCostSimplification(double pen0, double pen1, double pen2, double bestPrevCost)
    {
        if(minFrom(pen0, pen1 + P1, pen2 + P2) != minFrom(pen0, pen1 + P1, bestPrevCost + P2))
        {
            throw std::logic_error(std::string("File: ") + __FILE__ + ", line: " + std::to_string(__LINE__));
        }
    }

    // TODO: remove getCost and call costComp.getCost directly after ensuring that its clamped
    double getCost(Point2 basePixel, Point2 matchedPixel)
    {
        // matchedPixel.x = std::max(0, std::min(matchedPixel.x, cols));
        // matchedPixel.y = std::max(0, std::min(matchedPixel.y, rows));
        return costComp.getCost(basePixel, matchedPixel);
    }

    void findDisparities()
    {
		currentState = "Run: Disparities";
        // 3rd run: compute final disparity based on paths' bests
        // Set disparity to be weighted average of best disparities
        for(int r = 0; r < rows; ++r)
        {
            for(int c = 0; c < cols; ++c)
            {
				if (shouldTerminate)
					return;

                currentPixel = {r, c};
                matched = currentPixel;

                for(int i = 0; i < SgmPathsManager::pathsCount; ++i)
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
};

template<typename Image_, typename CostComputer_>
void SgmCostAggregator<Image_, CostComputer_>::setParameters(SgmParameters& params)
{

}
}
