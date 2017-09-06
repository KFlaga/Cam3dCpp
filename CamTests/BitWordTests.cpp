#include <CamCommon\BitWord.hpp>
#include <CamCommon\Profiler.hpp>
#include <gtest\gtest.h>


namespace CamTests
{		

	template<typename BitWord>
	static void PerformanceTest()
	{
		const int size = 500;
		BitWord* map0 = new BitWord[size * size];
		BitWord* map1 = new BitWord[size * size];

		for (int x = 0; x < size; ++x)
		{
			for (int y = 0; y < size; ++y)
			{
				map0[y * size + x] = BitWord{};
				map1[y * size + x] = BitWord{};
			}
		}

		double totalCost = 0;
		for (int i = 0; i < 100; ++i)
		{
			for (int y = 0; y < size; ++y)
			{
				for (int x = 0; x < size; ++x)
				{
					totalCost += map0[y * size + x].getHammingDistance(map1[y * size + x]);
				}
			}
		}
	}
	
	template<typename BitWord>
	static void DistanceTest()
	{
		BitWord bw1{};]
		for(int i = 0; i < BitWord::wordLength; ++i)
		{
			bw1.bytes[i] = (BitWord::uintType)-1;
		}
		BitWord bw2{};
		ASSERT_EQ(sizeof(BitWord) * 8 , bw1.getHammingDistance(bw2));
	}

	TEST(TestBitWords, Bitword32_Performance)
	{
		PerformanceTest<BitWord32<8>>();
	}
	
	TEST(TestBitWords, Bitword32_GetDistance)
	{
		DistanceTest<BitWord32<2>>();
	}

	TEST(TestBitWords, Bitword64_Performance)
	{
		PerformanceTest<BitWord64<4>>();
	}

	TEST(TestBitWords, Bitword64_GetDistance)
	{
		DistanceTest<BitWord64<2>>();
	}
}