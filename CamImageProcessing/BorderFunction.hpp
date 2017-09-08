#pragma once

#include <functional>

namespace cam3d
{
    // Defines generic operation on image pixels with different operations if
    // pixel is on image border
    class BorderFunction
    {
    public:
        using FunctionType = std::function<void(int pixRow, int pixCol)>;

        // Executes 'mainFun' on pixels in range (y:[bh,rows-bh], x:[bw,cols-bw]) and
        // 'borderFun' for rest of pixels (on border)
        
        static void run(FunctionType mainFun, FunctionType borderFun,
            int borderWidth, int borderHeight, int rows, int cols)
        {
            int maxX = cols - borderWidth;
            int maxY = rows - borderHeight;
            for(int y = borderHeight; y < maxY; ++y)
            {
                for(int x = borderWidth; x < maxX; ++x)
                {
                    mainFun(y, x);
                }
            }

            // 1) Top border
            for(int y = 0; y < borderHeight; ++y)
                for(int x = 0; x < cols; ++x)
                    borderFun(y, x);
            // 2) Right border
            for(int y = borderHeight; y < rows; ++y)
                for(int x = cols - borderWidth; x < cols; ++x)
                    borderFun(y, x);
            // 3) Bottom border
            for(int y = rows - borderHeight; y < rows; ++y)
                for(int x = 0; x < maxX; ++x)
                    borderFun(y, x);
            // 4) Left border
            for(int y = borderHeight; y < maxY; ++y)
                for(int x = 0; x < borderWidth; ++x)
                    borderFun(y, x);
        }
    };
}
