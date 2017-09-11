#pragma once

#include "PreReqs.hpp"
#include "Vector2.hpp"
#include <vector>
#include <cstring>

namespace cam3d
{
    // Represent static 2d array in row-major order
    // TODO: change name to Array2d
    template<typename T, int rows_, int cols_>
    class Array2d
    {
    protected:
#ifdef _DEBUG
        std::vector<std::vector<T>> data;
#else
        T data[rows][cols];
#endif
    public:
        static constexpr int rows = rows_;
        static constexpr int cols = cols_;

#ifdef _DEBUG
        Array2d()
        { 
            data.resize(rows);
            for(int r = 0; r < rows; ++r)
            {
                data[r].resize(cols);
            }
        }
#else // _RELEASE
        Matrix()
        {

        }
#endif
        T& operator()(const int r, const int c) { return data[r][c]; }
        const T& operator()(const int r, const int c) const { return data[r][c]; }

        T& operator()(const Vector2i& pos) { return data[pos.y][pos.x]; }
        const T& operator()(const Vector2i& pos) const { return data[pos.y][pos.x]; }

        // Zeroes array ( only for numerics/pointers )
        void clear()
        {
            for(int r = 0; r < rows; ++r)
            {
                for(int c = 0; c < cols; ++c)
                {
                    data[r][c] = T{};
                }
            }
        }
        // Fills array with given value
        void fill(const T& value)
        {
            for(int r = 0; r < rows; ++r)
            {
                for(int c = 0; c < cols; ++c)
                {
                    data[r][c] = value;
                }
            }
        }

        int getRowCount() const { return rows; }
        int getColumnCount() const { return cols; }
    };
}
