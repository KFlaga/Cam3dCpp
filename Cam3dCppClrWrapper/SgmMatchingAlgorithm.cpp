#include "Stdafx.h"
#include "SgmMatchingAlgorithm.h"
#include <type_traits>

namespace Cam3dWrapper
{
	SgmMatchingAlgorithm::SgmMatchingAlgorithm()
	{
		sgm = nullptr;
	}

	SgmMatchingAlgorithm::~SgmMatchingAlgorithm()
	{
		if (sgm != nullptr)
		{
			delete sgm;
			sgm = nullptr;
		}
	}

	void SgmMatchingAlgorithm::Process(SgmParameters^ parameters_)
	{
		parameters = parameters_;

		if (sgm != nullptr)
		{
			delete sgm;
		}

		mapLeftToRight = gcnew DisparityMapWrapper{ parameters->rows, parameters->cols };
		mapRightToLeft = gcnew DisparityMapWrapper{ parameters->rows, parameters->cols };

		sgm = createSgm(parameters, true);
		sgm->computeMatchingCosts(parameters->toNative());
		mapLeftToRight->updateNative();
		delete sgm;

		sgm = createSgm(parameters, false);
		sgm->computeMatchingCosts(parameters->toNative());
		mapRightToLeft->updateNative();
		delete sgm;

		sgm = nullptr;
	}

	void SgmMatchingAlgorithm::Terminate()
	{
		if (sgm != nullptr)
		{
			sgm->terminate();
		}
	}

	System::String^ SgmMatchingAlgorithm::GetStatus()
	{
		if (sgm != nullptr)
		{
			std::string status = sgm->getState();
			System::String^ res = gcnew System::String(status.c_str());

			return res;
		}
		return gcnew System::String("");
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

	cam3d::ISgmCostAggregator* SgmMatchingAlgorithm::createSgm(SgmParameters^ parameters, bool isLeftBase)
	{
		parameters->maskRadius = parameters->maskRadius > 7 ? 7 : parameters->maskRadius;
		DisparityMapWrapper^ map = isLeftBase ? mapLeftToRight : mapRightToLeft;
		cam3d::ISgmCostAggregator* sgm = nullptr;
		if (parameters->imageType == ImageType::Grey)
		{
			sgm = SgmCreator<1, 8, cam3d::GreyScaleImage>::createSgm(parameters, isLeftBase, map);
		}
		else if (parameters->imageType == ImageType::MaskedGrey)
		{
			using MaskedImage = cam3d::MaskedImage<cam3d::GreyScaleImage>;
			sgm = SgmCreator<1, 8, MaskedImage>::createSgm(parameters, isLeftBase, map);
		}
		else
		{
			throw std::invalid_argument("Only GrayScaleImage or masked GreyScaleImage supported");
		}
		return sgm;
	}

	cam3d::SgmParameters SgmParameters::toNative()
	{
        cam3d::SgmParameters p;
		p.censusMaskRadius = this->maskRadius;
		p.lowPenaltyCoeff = this->lowPenaltyCoeff;
		p.highPenaltyCoeff = this->highPenaltyCoeff;
		p.gradientCoeff = this->gradientCoeff;
        p.disparityCostMethod = (cam3d::CostMethod)this->disparityCostMethod;
        p.disparityMeanMethod = (cam3d::MeanMethod)this->disparityMeanMethod;
		p.diparityPathLengthThreshold = this->diparityPathLengthThreshold;
		return p;
	}
}
