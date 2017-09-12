#pragma once

#include "PreReqs.hpp"
#include "Vector2.hpp"
#include <vector>
#include <cstring>

namespace cam3d
{
    // Represent static 3d array: [row][col][dim]
    template<typename T, int rows_, int cols_, int dim_>
    class Array3d
    {
    public:
        static constexpr int rows = rows_;
        static constexpr int cols = cols_;
        static constexpr int dim = dim_;
    protected:
        T data[rows][cols][dim];

    public:

        T& operator()(const int r, const int c, const int d)
        {
            return data[r][c][d];
        }
        const T operator()(const int r, const int c, const int d) const
        {
            return data[r][c][d];
        }
        T& operator()(const Point2 p, const int d)
        {
            return data[p.y][p.x][d];
        }
        const T operator()(const Point2 p, const int d) const
        {
            return data[p.y][p.x][d];
        }
        T* operator()(const int r, const int c)
        {
            return &data[r][c][0];
        }
        const T* operator()(const int r, const int c) const
        {
            return &data[r][c][0];
        }
        T* operator()(const Point2 p)
        {
            return &data[p.y][p.x][0];
        }
        const T* operator()(const Point2 p) const
        {
            return &data[p.y][p.x][0];
        }

        // Fills array with default objects
        void clear()
        {
            for(int r = 0; r < rows; ++r)
            {
                for(int c = 0; c < cols; ++c)
                {
                    for(int d = 0; d < dim; ++d)
                    {
                        data[r][c][d] = T{};
                    }
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
                    for(int d = 0; d < dim; ++d)
                    {
                        data[r][c][d] = value;
                    }
                }
            }
        }

        int getRowCount() const { return rows; }
        int getColumnCount() const { return cols; }
        int getDimCount() const { return dim; }
    };
}
