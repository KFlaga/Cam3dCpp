#pragma once

#include <Cam3dCppClrWrapper\GreyScaleImageWrapper.h>
#include <Cam3dCppClrWrapper\MaskedImageWrapper.h>
#include <Cam3dCppClrWrapper\DisparityMapWrapper.h>
#include <CamImageProcessing\SgmCostAggregator.hpp>
#include <CamImageProcessing\SgmDisparityComputer.hpp>
#include "Wrapper.h"

namespace Cam3dWrapper
{
	public enum class ImageType : int
	{
        Grey = 0,
		Color = 100,
        MaskedGrey = 1,
		MaskedColor = 101
	};

	public enum class DisparityCostMethod : int
	{
		DistanceToMean = (int)cam3d::CostMethod::DistanceToMean,
		DistanceSquredToMean = (int)cam3d::CostMethod::DistanceSquredToMean,
		DistanceSquredToMeanRoot = (int)cam3d::CostMethod::DistanceSquredToMeanRoot,
	};

	public enum class DisparityMeanMethod : int
	{
		SimpleAverage = (int)cam3d::MeanMethod::SimpleAverage,
		WeightedAverage = (int)cam3d::MeanMethod::WeightedAverage,
		WeightedAverageWithPathLength = (int)cam3d::MeanMethod::WeightedAverageWithPathLength,
	};

	// TODO: expose struct with all parameters to c#
	public ref class SgmParameters
	{
	public:
		ImageType imageType;
		IWrapper^ leftImageWrapper;
		IWrapper^ rightImageWrapper;

		int rows;
		int cols;

		double lowPenaltyCoeff;
		double highPenaltyCoeff;
		double gradientCoeff;

		int maskRadius;

		DisparityCostMethod disparityCostMethod;
		DisparityMeanMethod disparityMeanMethod;
		double diparityPathLengthThreshold;

	internal:
		cam3d::SgmParameters toNative();
	};

	public ref class SgmMatchingAlgorithm
	{
	public:
		SgmMatchingAlgorithm();
		~SgmMatchingAlgorithm();

		DisparityMapWrapper^ GetMapLeft() { return mapLeftToRight; }
		DisparityMapWrapper^ GetMapRight() { return mapRightToLeft; }

		void Process(SgmParameters^ parameters);
		void Terminate();
		System::String^ GetStatus();

	private:
		cam3d::ISgmCostAggregator* createSgm(SgmParameters^ parameters, bool isLeftBase);

		DisparityMapWrapper^ mapLeftToRight;
		DisparityMapWrapper^ mapRightToLeft;
		SgmParameters^ parameters;
		cam3d::ISgmCostAggregator* sgm;
	};
}
