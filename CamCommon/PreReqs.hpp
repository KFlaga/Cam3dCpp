#pragma once

#include <stdint.h>
#include <limits.h>
#include <string>
#include <cmath>
#include <limits>

#if defined(__x86_64__) || defined(_M_X64)
#define _64bit
#else
#define _32bit
#endif

namespace cam3d
{
#if defined(_64bit)
	typedef int64_t Int;
	typedef uint64_t UInt;
#else
	typedef int32_t Int;
	typedef uint32_t UInt;
#endif

	typedef std::string String;

	inline bool equals(double a, double b, double delta = 1e-6)
	{
		return std::abs(a - b) < delta;
	}
	inline bool notEquals(double a, double b, double delta = 1e-6)
	{
		return std::abs(a - b) > delta;
    }

    namespace global
    {
        constexpr int imageRows = 480;
        constexpr int imageCols = 640;
    }
}
