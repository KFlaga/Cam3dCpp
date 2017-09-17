find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

set(SOURCES_CamTests 
   TestBitWord.cpp
   TestVector2.cpp
   TestCensus.cpp
   TestSgmPaths.cpp
   TestSgmPathsManager.cpp
   TestSgmAggregator.cpp
   TestSgmDisparityComputer.cpp
)

add_executable(tests 
    gtestmain.cpp
    ${SOURCES_CamTests}
    )

target_link_libraries(tests 
    ${GTEST_BOTH_LIBRARIES}
)
