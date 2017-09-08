#pragma once

#include <CamCommon/Matrix.hpp>
#include <type_traits>

namespace cam3d
{
template<typename Image, int rows, int cols, bool haveRefImage = true>
class MaskedImage
{
public:
    using Matrix = Image::Matrix;
    using ImageFieldType = std::conditional<haveRefImage, Image&, Image>::type;
    static constexpr int channels = Image::channels;

private:
    ImageFieldType image;
    Matrix<bool, Image::Matrix::rows, Image::Matrix::cols> mask;

public:
    MaskedImage(ImageFieldType image_) :
        image(image_),
        mask{ }
    {
        mask.fill(false);
    }

    double operator()(int y, int x) const { return image(y, x); }
    double& operator()(int y, int x) { return image(y, x); }
    double operator()(int y, int x, int channel) const { return image(y, x, channel); }
    double& operator()(int y, int x, int channel) { return image(y, x, channel); }

    int getColumnCount() const { return image.getColumnCount(); }
    int getRowCount() const { return image.getRowCount(); }
    int getChannelsCount() const { return channels; }

    bool haveValueAt(int y, int x) { return mask(y, x); }
    void setMaskAt(int y, int x, bool value) { mask(y, x) = value; }

    Matrix& getMatrix(int channel) { return image.getMatrix(channel); }
    const Matrix& getMatrix(int channel) const { return image.getMatrix(channel); }
    Matrix& getMatrix() { return image.getMatrix(); }
    const Matrix& getMatrix() const { return image.getMatrix(); }
};
}
