#pragma once

#include <CamCommon/Matrix.hpp>

namespace cam3d
{
    class GreyScaleImage
    {
    private:
        Matrix<double> imageMatrix[3];

    public:
        double operator()(int y, int x) const { return imageMatrix(y,x); }
        double& operator()(int y, int x) { return imageMatrix(y,x); }
        double operator()(int y, int x, int channel) const { return imageMatrix(y,x); }
        double& operator()(int y, int x, int channel) { return imageMatrix(y,x); }

        int getColumnCount() const { return imageMatrix.getColumnCount(); }
        int getRowCount() const { return imageMatrix.getRowCount(); }

        bool haveValueAt(int y, int x) { return true; }

        Matrix<double>& getMatrix(int channel) { return imageMatrix; }
        const Matrix<double>& getMatrix(int channel) const { return imageMatrix; }
        Matrix<double>& getMatrix() { return imageMatrix; }
        const Matrix<double>& getMatrix() const { return imageMatrix; }
    };
}