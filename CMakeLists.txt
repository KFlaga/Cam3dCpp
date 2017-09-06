project(cam3d)
#set c++14 / debug build

set(BINARIES_DIR ${CMAKE_PROJECT_DIR}/bin)
set(LIBRARIES_DIR ${CMAKE_PROJECT_DIR}/lib)

include_directories( ${CMAKE_PROJECT_DIR} )

add_subdirectory(CamCommon)
add_subdirectory(CamImageProcessing)
add_subdirectory(CamTests)