#pragma once

#include <Cam3dCppClrWrapper\GreyScaleImageWrapper.h>
#include <Cam3dCppClrWrapper\MaskedImageWrapper.h>
#include <Cam3dCppClrWrapper\DisparityMapWrapper.h>
#include <CamImageProcessing\SgmCostAggregator.hpp>
#include "Wrapper.h"

namespace Cam3dWrapper
{
	public enum class ImageType
	{
		Grey = 0,
		Color,
		MaskedGrey,
		MaskedColor
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
	};

	public ref class MatchingAlgorithmWrapper
	{
	public:
		MatchingAlgorithmWrapper();
		~MatchingAlgorithmWrapper();

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