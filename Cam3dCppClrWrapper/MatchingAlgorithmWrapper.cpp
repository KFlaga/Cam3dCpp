#include "Stdafx.h"
#include "MatchingAlgorithmWrapper.h"
#include <type_traits>

namespace Cam3dWrapper
{
	MatchingAlgorithmWrapper::MatchingAlgorithmWrapper()
	{
		sgm = nullptr;
	}

	MatchingAlgorithmWrapper::~MatchingAlgorithmWrapper()
	{
		if (sgm != nullptr)
		{
			delete sgm;
			sgm = nullptr;
		}
	}

	void MatchingAlgorithmWrapper::Process(SgmParameters^ parameters_)
	{
		parameters = parameters_;

		if (sgm != nullptr)
		{
			delete sgm;
		}

		mapLeftToRight = gcnew DisparityMapWrapper{ parameters->rows, parameters->cols };
		mapRightToLeft = gcnew DisparityMapWrapper{ parameters->rows, parameters->cols };

		sgm = createSgm(parameters, true);
		sgm->computeMatchingCosts();
		mapLeftToRight->updateNative();
		delete sgm;

		sgm = createSgm(parameters, false);
		sgm->computeMatchingCosts();
		mapRightToLeft->updateNative();
		delete sgm;

		sgm = nullptr;
	}

	void MatchingAlgorithmWrapper::Terminate()
	{
		sgm->terminate();
	}

	System::String^ MatchingAlgorithmWrapper::GetStatus()
	{
		std::string status = sgm->getState();
		System::String^ res = gcnew System::String(status.c_str());

		return res;
	}

	template<int maskRadius, int maxRadiusPlusOne, typename ImageT>
	struct SgmCreator
	{
		static cam3d::ISgmCostAggregator* createSgm(SgmParameters^ parameters, bool isLeftBase, DisparityMapWrapper^ map)
		{
			if (maskRadius == parameters->maskRadius)
			{
				return new cam3d::SgmCostAggregator<ImageT, cam3d::CensusCostComputer32<ImageT, maskRadius>>{
					parameters->rows,
					parameters->cols,
					isLeftBase,
					isLeftBase ? *parameters->leftImageWrapper->getNativeAs<ImageT>() : *parameters->rightImageWrapper->getNativeAs<ImageT>(),
					!isLeftBase ? *parameters->leftImageWrapper->getNativeAs<ImageT>() : *parameters->rightImageWrapper->getNativeAs<ImageT>(),
					*map->getNativeAs<cam3d::DisparityMap>()
				};
			}
			else
			{
				return SgmCreator<maskRadius + 1, maxRadiusPlusOne, ImageT>::createSgm(parameters, isLeftBase, map);
			}
		}
	};

	template<int rmax, typename ImageT>
	struct SgmCreator<rmax, rmax, ImageT>
	{
		static cam3d::ISgmCostAggregator* createSgm(SgmParameters^ parameters, bool isLeftBase, DisparityMapWrapper^ map)
		{
			throw std::invalid_argument(std::string("Census mask radius must be in range [1, ") + std::to_string(rmax - 1) + "].");
		}
	};

	cam3d::ISgmCostAggregator* MatchingAlgorithmWrapper::createSgm(SgmParameters^ parameters, bool isLeftBase)
	{
		parameters->maskRadius = parameters->maskRadius > 7 ? 7 : parameters->maskRadius;
		DisparityMapWrapper^ map = isLeftBase ? mapLeftToRight : mapRightToLeft;
		if (parameters->imageType == ImageType::Grey)
		{
			return SgmCreator<1, 8, cam3d::GreyScaleImage>::createSgm(parameters, isLeftBase, map);
		}
		else if (parameters->imageType == ImageType::MaskedGrey)
		{
			using MaskedImage = cam3d::MaskedImage<cam3d::GreyScaleImage>;
			return SgmCreator<1, 8, MaskedImage>::createSgm(parameters, isLeftBase, map);
		}
		else
		{
			throw std::invalid_argument("Only GrayScaleImage or masked GreyScaleImage supported");
		}
	}
}
