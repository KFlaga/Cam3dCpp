#pragma once

#include <CamCommon/Matrix.hpp>

namespace cam3d
{
template<int rows, int cols>
class GreyScaleImage
{
public:
    using Matrix = Matrix<double, rows, cols>;
    static constexpr int channels = 1;

private:
    Matrix imageMatrix;

public:
    GreyScaleImage() = default;

    double operator()(int y, int x) const { return imageMatrix(y,x); }
    double& operator()(int y, int x) { return imageMatrix(y,x); }
    double operator()(int y, int x, int channel) const { return imageMatrix(y,x); }
    double& operator()(int y, int x, int channel) { return imageMatrix(y,x); }

    int getColumnCount() const { return imageMatrix.getColumnCount(); }
    int getRowCount() const { return imageMatrix.getRowCount(); }
    int getChannelsCount() const { return channels; }

    bool haveValueAt(int y, int x) { return true; }

    Matrix& getMatrix(int channel) { return imageMatrix; }
    const Matrix& getMatrix(int channel) const { return imageMatrix; }
    Matrix& getMatrix() { return imageMatrix; }
    const Matrix& getMatrix() const { return imageMatrix; }
};
}
