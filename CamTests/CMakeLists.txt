
find_package(GTest REQUIRED)
#include gtest

set(SOURCES_CamTests 
	TestBitWord.cpp
)

add_executable(BINARIES_DIR/tests 
	gtestmain.cpp
	SOURCES_CamTests
	)

target_link_libraries(BINARIES_DIR/tests 
	${GTEST_BOTH_LIBRARIES}
	LIBRARIES_DIR/CamCommon.a
	LIBRARIES_DIR/CamImageProcessing.a
)