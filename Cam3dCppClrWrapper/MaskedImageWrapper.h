#pragma once

#include <CamImageProcessing\MaskedImage.hpp>
#include <Cam3dCppClrWrapper\GreyScaleImageWrapper.h>
#include <Cam3dCppClrWrapper\ColorImageWrapper.h>
#include "Wrapper.h"

namespace Cam3dWrapper
{
	template<typename ImageT>
	public ref class MaskedImageWrapper : public Wrapper<cam3d::MaskedImage<ImageT>>
	{
	public:
		MaskedImageWrapper(Wrapper<ImageT>^ image_) :
			image(image_)
		{
			mask = gcnew cli::array<bool, 2>(image->GetRows(), image->GetCols());
			native = new cam3d::MaskedImage<ImageT>(*image->getNativeAs<ImageT>());
			updateNative();
		}

		~MaskedImageWrapper()
		{
			delete native;
		}

		Wrapper<ImageT>^ GetImage()
		{
			return image;
		}

		cli::array<bool, 2>^ GetMask()
		{
			return mask;
		}

		void SetMask(cli::array<bool, 2>^ mask_)
		{
			mask = mask_;
			Update();
		}

		void Update();

		int GetRows() { return image->GetRows(); }
		int GetCols() { return image->GetCols(); }

	internal:
		virtual void updateNative() override;

	private:
		cli::array<bool, 2>^ mask;
		Wrapper<ImageT>^ image;
	};

	using GreyMaskedImageWrapper = MaskedImageWrapper<cam3d::GreyScaleImage>;
	using ColorMaskedImageWrapper = MaskedImageWrapper<cam3d::ColorImage>;

	template<typename ImageT>
	void MaskedImageWrapper<ImageT>::Update()
	{
		image->Update();
		for (int r = 0; r < rows; ++r)
		{
			for (int c = 0; c < cols; ++c)
			{
				native->setMaskAt(r, c , mask[r, c]);
			}
		}
	}

	template<typename ImageT>
	void MaskedImageWrapper<ImageT>::updateNative()
	{
		image->updateNative();
		for (int r = 0; r < rows; ++r)
		{
			for (int c = 0; c < cols; ++c)
			{
				matrix[r, c] = native->haveValueAt(r, c);
			}
		}
	}
}