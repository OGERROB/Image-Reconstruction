cmake_minimum_required( VERSION 2.8 )
project( fangshe )

set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )



find_package( OpenCV 3.1 REQUIRED )
# find_package( OpenCV REQUIRED ) # use this if in OpenCV2 
add_library(libLZY functions.cpp)

include_directories( 
    ${OpenCV_INCLUDE_DIRS} 
    "/usr/include/eigen3/"
${PROJECT_SOURCE_DIR}
)


add_executable( fangshe fangshe.cpp )
target_link_libraries( fangshe 
   ${OpenCV_LIBS}
libLZY
)
