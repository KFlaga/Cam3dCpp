#include <CamCommon\Vector2.hpp>
#include <gtest\gtest.h>

namespace cam3d
{
    TEST(TestVector2, ConversionIntDoubleWorks)
    {
        Vector2f v1;
        Vector2i v2;
        Vector2f v3 = v2 + Vector2i(v1);
        bool x = v1 == v2;
    }
    
    TEST(TestVector2, ConversionIntUnitWorks)
    {
        Vector2u v1;
        Vector2i v2;
        Vector2u v3 = v2 + Vector2i(v1);
        bool x = v1 == v2;
    }
}