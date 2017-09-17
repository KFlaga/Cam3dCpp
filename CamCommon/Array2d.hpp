#pragma once

#include "PreReqs.hpp"
#include "Vector2.hpp"
#include <vector>
#include <algorithm>

namespace cam3d
{
    template<typename T>
    class Array2d
    {
    protected:
        int rows;
        int cols;

    public:
        int getRowCount() const { return rows; }
        int getColumnCount() const { return cols; }

#ifdef _DEBUG
    private:
        std::vector<std::vector<T>> data;

    public:
        Array2d(int rows_, int cols_) : rows{rows_}, cols{cols_}
        { 
            data.resize(rows);
            for(int r = 0; r < rows; ++r)
            {
                data[r].resize(cols);
            }
        }

        T& operator()(const int r, const int c) { return data[r][c]; }
        const T operator()(const int r, const int c) const { return data[r][c]; }

        T& operator()(const Vector2i& pos) { return data[pos.y][pos.x]; }
        const T operator()(const Vector2i& pos) const { return data[pos.y][pos.x]; }

        void clear()
        {
            fill(T{});
        }

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

#else
    private:
        int getSize() const { return rows * cols; }
        int getIdx(const int r, const int c) const { return r * cols + c; }
        std::vector<T> data;

    public:
        Array2d(int rows_, int cols_) : rows{rows_}, cols{cols_}
        {
            data.resize(getSize());
        }

        T& operator()(const int r, const int c) { return data[getIdx(r, c)]; }
        const T operator()(const int r, const int c) const { return data[getIdx(r, c)]; }

        T& operator()(const Vector2i& pos) { return data[getIdx(pos.y, pos.x)]; }
        const T operator()(const Vector2i& pos) const { return data[getIdx(pos.y, pos.x)]; }

        void clear()
        {
            fill(T{});
        }

        void fill(const T& value)
        {
            for(int i = 0, size = getSize(); i < size; ++ i)
            {
                data[i] = value();
            }
        }
#endif
    };
}
