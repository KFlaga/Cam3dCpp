#pragma once

#include <CamCommon/DisparityMap.hpp>
#include <CamImageProcessing/GreyScaleImage.hpp>
#include <CamImageProcessing/CensusCostComputer.hpp>
#include <CamCommon/BitWord.hpp>
//#include <CamImageProcessing/MatchConfidenceComputer.hpp>
#include <array>
#include <vector>
#include <functional>
#include <algorithm>

namespace cam3d
{
enum class MeanMethod
{
	SimpleAverage,
	WeightedAverage,
	WeightedAverageWithPathLength,
};

enum class CostMethod
{
	DistanceToMean,
	DistanceSquredToMean,
	DistanceSquredToMeanRoot,
};

template<typename Image_, typename CostComputer_>
class SgmDisparityComputerForTests
{
public:
    static constexpr int pathsCount = 8;

protected:
    int rows;
    int cols;

	using Image = Image_;
	using CostComputer = CostComputer_;
    using DisparityComputer = SgmDisparityComputerForTests;
    using DisparityList = std::array<Disparity, pathsCount>;

    Image& imageBase;
    Image& imageMatched;
    DisparityMap& disparityMap;
    CostComputer& costComp;

    DisparityList dispForPixel;
    int idx; // ?
    double pathLengthTreshold;

    using ComputeMeanFunc = double(DisparityComputer::*)(int,int);
    ComputeMeanFunc computeMean;
    using ComputeCostFunc = double(DisparityComputer::*)(double,int,int);
    ComputeCostFunc computeCost;

    MeanMethod meanMethod;
    CostMethod costMethod;

public:
    SgmDisparityComputerForTests(int rows_, int cols_, DisparityMap& map,
		Image& imageBase_, Image& imageMatched_, CostComputer& costComp_) :
        rows{rows_},
        cols{cols_},
        idx{0},
        disparityMap{map},
        imageBase{imageBase_},
        imageMatched{imageMatched_},
        costComp{costComp_}
    {
        setMeanMethod(MeanMethod::SimpleAverage);
        setCostMethod(CostMethod::DistanceToMean);
        std::fill(dispForPixel.begin(), dispForPixel.end(), Disparity{});
    }

    MeanMethod getMeanMethod() const;
    void setMeanMethod(MeanMethod method);
    CostMethod getCostMethod() const;
    void setCostMethod(CostMethod method);
	double getPathLengthTreshold() const;
	void setPathLengthTreshold(double val);

    DisparityMap& getDisparityMap() { return disparityMap; }

    void storeDisparity(Point2 pixelBase, Point2 pixelMatched, double cost)
    {
        storeDisparity(Disparity{pixelBase, pixelMatched, Disparity::Valid, cost, 0.0});
    }

    void storeDisparity(Disparity disp)
    {
#ifdef _DEBUG
        if(idx >= pathsCount)
        {
            throw std::logic_error(std::string("File: ") + __FILE__ + ", line: " + std::to_string(__LINE__));
        }
#endif
        dispForPixel[idx] = disp;
        ++idx;
    }

    void finalizeForPixel(Point2 pixelBase)
    {
        if(idx == 0)
        {
            // There was no disparity for pixel : set as invalid
            disparityMap(pixelBase) = Disparity{};
            return;
        }

        // 1) Sort by disparity
        std::sort(dispForPixel.begin(), dispForPixel.end(),
                  [](const Disparity d1, const Disparity d2){ return d1.dx < d2.dx; });
        disparityMap(pixelBase) = findBestDisparity(pixelBase);

        idx = 0;
    }

protected:
    Disparity findBestDisparity(Point2 pixelBase)
    {
        int start = 0;
        int count = idx;
        bool costLower = true;

        // 2) Find weighted mean m of disparities and distances s to m, s = ||m - d||
        // Cost function: C = sum(||m - d||) / n^2
        double mean = (this->*computeMean)(start, count);
        double cost = (this->*computeCost)(mean, start, count);
        while(costLower && count > 2) // 4) Repeat untill cost is minimised
        {
            // 3) Remove one disp from ends and check if cost is lower
            double mean1 = (this->*computeMean)(start + 1, count - 1);
            double cost1 = (this->*computeCost)(mean1, start + 1, count - 1);
            double mean2 = (this->*computeMean)(start, count - 1);
            double cost2 = (this->*computeCost)(mean2, start, count - 1);

            if(cost > cost1 || cost > cost2)
            {
                if(cost1 < cost2) // Remove first one -> move start by one pos
                {
                    start++;
                    cost = cost1;
                    mean = mean1;
                }
                else // Remove last one -> just decrement count
                {
                    cost = cost2;
                    mean = mean2;
                }
                count--;
                costLower = true;
            }
            else
            {
                costLower = false;
            }
        }

        cost = costComp.getCost(pixelBase, Point2{pixelBase.y, pixelBase.x + round(mean)});
        return Disparity{round(mean), Disparity::Valid, mean, cost, findConfidence(count, cost)};
    }

    double findConfidence(int count, double cost)
    {
        return ((double)count / (double)idx);
    }

    double findMean_Simple(int start, int count)
    {
        double mean = 0.0;
        for(int i = 0; i < count; ++i)
        {
            mean += dispForPixel[start + i].dx;
        }
        return mean / count;
    }

    double findMean_Weighted(int start, int count)
    {
        double mean = 0.0, wsum = 0.0, w = 0.0;
        for(int i = 0; i < count; ++i)
        {
            w = 1.0 / (dispForPixel[start + i].confidence + 1.0);
            wsum += w;
            mean += w * dispForPixel[start + i].dx;
        }
        return mean / wsum;
    }

    double findMean_WeightedPath(int start, int count)
    {
        double mean = 0.0;

        // 3) Same as 2 but includes path length
        double wsum = 0.0;
        double w;
        double pathLength = 1.0; // TODO: how to get this
        for(int i = 0; i < count; ++i)
        {
            w = std::min(1.0, pathLength * pathLengthTreshold) /
                (dispForPixel[start + i].confidence + 1.0);
            wsum += w;
            mean += w * dispForPixel[start + i].dx;
        }
        return mean / wsum;
    }

    double findCost_Simple(double mean, int start, int count)
    {
        double cost = 0.0;
        // 1) C = sum(||m - d||) / n^2
        for(int i = 0; i < count; ++i)
        {
            cost += std::abs(mean - dispForPixel[start + i].dx);
        }
        return cost / (count * count);
    }

    double findCost_Squared(double mean, int start, int count)
    {
        double cost = 0.0;
        // 2) C = sqrt(sum(||m - d||^2)) / n^2 -> same results as sum(||m - d||^2)/n^4
        double d;
        for(int i = 0; i < count; ++i)
        {
            d = mean - dispForPixel[start + i].dx;
            cost += d * d;
        }
        return cost / (count * count * count * count);
    }

    double findCost_Root(double mean, int start, int count)
    {
        double cost = 0.0;
        // 1) C = sum(||m - d||) / n*sqrt(n)
        for(int i = 0; i < count; ++i)
        {
            cost += std::abs(mean - dispForPixel[start + i].dx);
        }
        return cost / (count * std::sqrt(count));
    }
};

template<typename IT, typename CC>
MeanMethod SgmDisparityComputerForTests<IT, CC>::getMeanMethod() const
{
    return meanMethod;
}

template<typename IT, typename CC>
void SgmDisparityComputerForTests<IT, CC>::setMeanMethod(MeanMethod method)
{
    meanMethod = method;
    switch(method)
    {
    case MeanMethod::WeightedAverageWithPathLength:
        computeMean = &SgmDisparityComputerForTests::findMean_WeightedPath;
        break;
    case MeanMethod::WeightedAverage:
        computeMean = &SgmDisparityComputerForTests::findMean_Weighted;
        break;
    case MeanMethod::SimpleAverage:
    default:
        computeMean = &SgmDisparityComputerForTests::findMean_Simple;
        break;
    }
}

template<typename IT, typename CC>
CostMethod SgmDisparityComputerForTests<IT, CC>::getCostMethod() const
{
    return costMethod;
}

template<typename IT, typename CC>
void SgmDisparityComputerForTests<IT, CC>::setCostMethod(CostMethod method)
{
    costMethod = method;
    switch(method)
    {
    case CostMethod::DistanceSquredToMeanRoot:
        computeCost = &SgmDisparityComputerForTests::findCost_Root;
        break;
    case CostMethod::DistanceSquredToMean:
        computeCost = &SgmDisparityComputerForTests::findCost_Squared;
        break;
    case CostMethod::DistanceToMean:
    default:
        computeCost = &SgmDisparityComputerForTests::findCost_Simple;
        break;
    }
}

template<typename IT, typename CC>
double SgmDisparityComputerForTests<IT, CC>::getPathLengthTreshold() const
{
	return pathLengthTreshold;
}

template<typename IT, typename CC>
void SgmDisparityComputerForTests<IT, CC>::setPathLengthTreshold(double val)
{
	pathLengthTreshold = val;
}

}
