#include <CamCommon\BitWord.hpp>
#include <CamCommon\Profiler.hpp>
#include <gtest\gtest.h>

namespace cam3d
{
#ifdef _DEBUG
    constexpr int testIterations = 10;
#else
    constexpr int testIterations = 1000;
#endif

#ifdef _64bit
    std::string arch{ "64 bit" };
#else
    std::string arch{ "32 bit" };
#endif

    template<typename BitWord>
    static void PerformanceTest()
    {
        cam3d::profiling::Microseconds fullTime;
        cam3d::profiling::Microseconds iterationTime;
        {
            cam3d::profiling::ProfileScope p1{&fullTime};

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

            {
                cam3d::profiling::ProfileScope p2{&iterationTime};
                double totalCost = 0;
                for (int i = 0; i < testIterations; ++i)
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
        }
        std::cout << "Profiling (arch: " << arch << ")" << std::endl;
        std::cout << "  full time [ms]: " << fullTime / 1000 << std::endl;
        std::cout << "  iter time [ms]: " << iterationTime / 1000 << std::endl;
    }

    template<typename BitWord>
    static void DistanceTest()
    {
        BitWord bw1{};
        for(size_t i = 0; i < BitWord::lengthInWords; ++i)
        {
            bw1.bytes[i] = (typename BitWord::uint_t)-1;
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
