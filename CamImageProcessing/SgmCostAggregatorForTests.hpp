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
    double lowPenaltyCoeff;
    double highPenaltyCoeff;
    double gradientCoeff;
	// TODO: disparity range limits

    int censusMaskRadius;

    MeanMethod disparityMeanMethod;
    CostMethod disparityCostMethod;
    double diparityPathLengthThreshold;
};

class ISgmCostAggregator
{
public:
    virtual void computeMatchingCosts(SgmParameters params) = 0;
	virtual void terminate() = 0;
    virtual std::string getState() = 0;
	virtual DisparityMap& getDisparityMap() = 0;
	virtual Point2 getCurrentPixel() = 0;
};

template<typename Image_, typename CostComputer_>
class SgmCostAggregatorForTests : public ISgmCostAggregator
{
public:
    using Image = Image_;
	using CostComputer = CostComputer_;
	using DisparityComputer = SgmDisparityComputer<Image, CostComputer>;
    static constexpr int pathsCount = SgmPathsManager::pathsCount;

private:
    int rows;
    int cols;
    double lowPenaltyCoeff; // P1 = coeff * MaxCost
    double highPenaltyCoeff; // P2 = coeff * MaxCost * (1 - grad * |Ib - Im|)
    double gradientCoeff;
    bool isLeftImageBase = true;
    std::vector<double> thisStepCosts;

    using DisparityCost = typename SgmPathsManager::DisparityCost;
    using RunDirection = typename SgmPathsManager::RunDirection;
    SgmPathsManager pathMgr;

    double P1;
    double P2;

	Image& imageBase;
	Image& imageMatched;
	DisparityMap& map;
    CostComputer costComp;
    DisparityComputer dispComp;

    Point2 currentPixel;
    Point2 matched;

	std::atomic_bool shouldTerminate;
	std::string currentState;

public:
    SgmCostAggregatorForTests(int rows_, int cols_, bool isLeftImageBase_, Image& imageBase_, Image& imageMatched_, DisparityMap& map_) :
        rows{rows_},
        cols{cols_},
        isLeftImageBase{isLeftImageBase_},
		imageBase{imageBase_},
		imageMatched{imageMatched_},
		map{map_},
        pathMgr{rows_, cols_, [this](Point2 p1, Point2 p2){ return this->getCost(p1, p2); }, isLeftImageBase},
        costComp{rows_, cols_},
		dispComp{rows_, cols_, map_, imageBase_, imageMatched_, costComp},
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

    void setParameters(SgmParameters params);
	void terminate() override { shouldTerminate = true; }
	std::string getState() override { return currentState; }

    void computeMatchingCosts(SgmParameters params) override
    {
        setParameters(params);
		shouldTerminate = false;
		currentState = "Init";
        costComp.init(imageBase, imageMatched);
        P1 = lowPenaltyCoeff * costComp.getMaxCost();
        P2 = highPenaltyCoeff * costComp.getMaxCost();

        pathMgr.init();
        thisStepCosts.resize(cols + 1);

        findCostsTopDown();
        findCostsBottomUp();
        findDisparities();
    }

    int getDispRange(Point2 pixel)
    {
        return isLeftImageBase ?
                    pixel.x + 1:
                    cols - pixel.x;
    }

    void findCostsTopDown()
    {
        for(int y = 0; y < rows; ++y)
        {
            for(int x = 0; x < cols; ++x)
            {
				currentState = "Run: TopDown { " + std::to_string(y) + ", " + std::to_string(x) + " }";
				if (shouldTerminate)
					return;
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
				currentState = "Run: BottomUp { " + std::to_string(y) + ", " + std::to_string(x) + " }";
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

        for(int d = 0; d < maxDisp; ++d)
        {
            matched.x = isLeftImageBase ? path->currentPixel.x - d : path->currentPixel.x + d;

            double cost = findCostForDisparity(
                              path->currentPixel, path, d, maxDisp, bestPrev.disparity, bestPrev.cost);
            thisStepCosts[d] = cost;

            if(bestCost > cost)
            {
                bestCost = cost;
                bestDisp = d;
            }
        }
        pathMgr.setBestPathCosts(path->currentPixel, pathIdx, {bestCost, bestDisp});
        std::copy(thisStepCosts.begin(), thisStepCosts.end(), path->lastStepCosts.begin());
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
        //TEST_checkCostSimplification(pen0, pen1, pen2, bestPrevCost);

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
        if(matchedPixel.x != std::max(0, std::min(matchedPixel.x, cols)))
        {
            throw std::logic_error(std::string("File: ") + __FILE__ + ", line: " + std::to_string(__LINE__));
        }
        if(matchedPixel.y != std::max(0, std::min(matchedPixel.y, rows)))
        {
            throw std::logic_error(std::string("File: ") + __FILE__ + ", line: " + std::to_string(__LINE__));
        }
        return costComp.getCost(basePixel, matchedPixel);
    }

    void findDisparities()
    {
        // 3rd run: compute final disparity based on paths' bests
        // Set disparity to be weighted average of best disparities
        for(int r = 0; r < rows; ++r)
        {
            for(int c = 0; c < cols; ++c)
            {
                currentState = "Run: Disparities { " + std::to_string(r) + ", " + std::to_string(c) + " }";
				if (shouldTerminate)
					return;

                currentPixel = {r, c};
                matched = currentPixel;

                for(int i = 0; i < pathsCount; ++i)
                {
                    int dx = pathMgr.getBestPathCosts(currentPixel, i).disparity * (isLeftImageBase ? -1 : 1);
                    matched.x = currentPixel.x + dx;
                    double matchCost = getCost(currentPixel, matched);
                    dispComp.storeDisparity(Disparity{
                        dx, Disparity::Valid, static_cast<double>(dx), matchCost, 0.0
                    });
                }
                dispComp.finalizeForPixel(currentPixel);
            }
        }
    }
};

template<typename Image_, typename CostComputer_>
void SgmCostAggregatorForTests<Image_, CostComputer_>::setParameters(SgmParameters params)
{
	this->lowPenaltyCoeff = params.lowPenaltyCoeff;
	this->highPenaltyCoeff = params.highPenaltyCoeff;
	this->gradientCoeff = params.gradientCoeff;
	this->costComp.setMaskWidth(params.censusMaskRadius);
	this->costComp.setMaskHeight(params.censusMaskRadius);
	this->dispComp.setCostMethod((cam3d::CostMethod)params.disparityCostMethod);
	this->dispComp.setMeanMethod((cam3d::MeanMethod)params.disparityMeanMethod);
	this->dispComp.setPathLengthTreshold(params.diparityPathLengthThreshold);
}
}
