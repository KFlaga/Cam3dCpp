#pragma once

#include <stdint.h>
#include <string>
#include <cmath>
#include <limits>

#if (sizeof(std::intptr_t) == 8)
#define _64bit
#else
#define _32bit
#endif

namespace cam3d
{
#if _64bit
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
}