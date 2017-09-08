#pragma once

#include <CamCommon/Matrix.hpp>
#include <CamImageProcessing/BorderFunction.hpp>

namespace cam3d
{
template<typename Image, typename BitWord>
class CensusCostComputer
{
public:
    using uint = typename BitWord::uintType;
    using BitWordMatrix = Matrix<BitWord, Image::Matrix::rows, Image::Matrix::cols>;

private:
    int borderWidth;
    int borderHeight;
    int maskLength;
    double maxCost;

    BitWordMatrix censusBase;
    BitWordMatrix censusMatched;

    int maskWidth; // Actual width is equal to maskWidth*2 + 1
    int maskHeight; // Actual height is equal to maskHeight*2 + 1

public:
    double getCost(Point2 pixelBase, Point2 pixelMatched)
    {
        return censusBase(pixelBase.y, pixelBase.x).getHammingDistance(
                    censusMatched(pixelMatched.y, pixelMatched.x));
    }

    double getCostOnBorder(Point2 pixelBase, Point2 pixelMatched)
    {
        return getCost(pixelBase, pixelMatched);
    }

    void init(Image& imageBase, Image& imageMatched)
    {
        // Transform images using census transform
        censusBase.clear();
        censusMatched.clear();

        maskLength = (2 * maskHeight + 1) * (2 * maskWidth + 1);
        borderHeight = maskHeight;
        borderWidth = maskWidth;
        maxCost = maskLength - 1;

        BorderFunction::run
        (
            [this, &imageBase, &imageMatched](int y, int x) { censusTransform(y, x, imageBase, imageMatched); },
            [this, &imageBase, &imageMatched](int y, int x) { censusTransform_Border(y, x, imageBase, imageMatched); },
            maskWidth, maskHeight,
            imageBase.getRowCount(), imageBase.getColumnCount()
        );
    }

    void update() { }

    int getBorderWidth() const { return borderWidth; }
    int getBorderHeight() const { return borderHeight; }
    int getMaskLength() const { return maskLength; }
    double getMaxCost() const { return maxCost; }

    const BitWordMatrix& getCensusBase() const { return censusBase; }
    const BitWordMatrix& getCensusMatched() const { return censusMatched; }

    int getMaskWidth() const { return maskWidth; }
    void setMaskWidth(int value) { maskWidth = value; }
    int getMaskHeight() const { return maskHeight; }
    void setMaskHeight(int value) { maskHeight = value; }

private:
    void censusTransform(int y, int x, Image& imageBase, Image& imageMatched)
    {
        uint maskBase[BitWord::lengthInWords];
        uint maskMatch[BitWord::lengthInWords];
        std::memset(maskBase, 0, sizeof(maskBase));
        std::memset(maskMatch, 0, sizeof(maskMatch));

        int dx, dy, maskPos = 0;
        for(dy = -maskHeight; dy <= maskHeight; ++dy)
        {
            for(dx = -maskWidth; dx <= maskWidth; ++dx)
            {
                if(imageBase(y + dy, x + dx) < imageBase(y, x))
                {
                    maskBase[maskPos / BitWord::bitSizeOfWord] |= (1u << (maskPos % BitWord::bitSizeOfWord));
                }
                if(imageMatched(y + dy, x + dx) < imageMatched(y, x))
                {
                    maskMatch[maskPos / BitWord::bitSizeOfWord] |= (1u << (maskPos % BitWord::bitSizeOfWord));
                }
                ++maskPos;
            }
        }

        censusBase(y, x) = BitWord{&maskBase[0]};
        censusMatched(y, x) = BitWord{&maskMatch[0]};
    }

    void censusTransform_Border(int y, int x, Image& imageBase, Image& imageMatched)
    {
        uint maskBase[BitWord::lengthInWords];
        uint maskMatch[BitWord::lengthInWords];
        std::memset(maskBase, 0, sizeof(maskBase));
        std::memset(maskMatch, 0, sizeof(maskMatch));

        int dx, dy, px, py, maskPos = 0;
        for(dy = -maskHeight; dy <= maskHeight; ++dy)
        {
            for(dx = -maskWidth; dx <= maskWidth; ++dx)
            {
                px = x + dx;
                px = px > imageBase.getColumnCount() - 1 ? 2 * imageBase.getColumnCount() - px - 2 : px;
                px = px < 0 ? -px : px;

                py = y + dy;
                py = py > imageMatched.getRowCount() - 1 ? 2 * imageMatched.getRowCount() - py - 2 : py;
                py = py < 0 ? -py : py;

                if(imageBase(py, px) < imageBase(y, x))
                {
                    maskBase[maskPos / BitWord::bitSizeOfWord] |= (1u << (maskPos % BitWord::bitSizeOfWord));
                }
                if(imageMatched(py, px) < imageMatched(y, x))
                {
                    maskMatch[maskPos / BitWord::bitSizeOfWord] |= (1u << (maskPos % BitWord::bitSizeOfWord));
                }
                ++maskPos;
            }
        }

        censusBase(y, x) = BitWord{&maskBase[0]};
        censusMatched(y, x) = BitWord{&maskMatch[0]};
    }
};
}
