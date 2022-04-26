cmake_minimum_required(VERSION 2.8)
project(ObjSLAM)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

# ======  start c++ compile  ======

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")
#compiling with march native causes seg fault when linked against gtsam
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)


find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

# This should eventually just be for release
#othrwise too many warnings I cannot see the build errors!!
add_definitions(-w)

find_package(Eigen3 3.1.0 REQUIRED)
find_package(CSparse REQUIRED)
find_package(Gflags REQUIRED)
find_package(Glog 0.3.5 REQUIRED)
find_package(GTSAM REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include

${EIGEN3_INCLUDE_DIR}
${CSPARSE_INCLUDE_DIR}
${GLOG_INCLUDE_DIRS} 
${GTSAM_INCLUDE_DIR}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/ORBextractor.cc
src/Converter.cc
src/Map.cc
src/Optimizer.cc
src/Frame.cc
src/FactorGraph.cc

src/utils/UtilsGTSAM.cc

src/cvplot/color.cc
src/cvplot/figure.cc
src/cvplot/highgui.cc
src/cvplot/window.cc
src/cvplot/internal.h
)

target_link_libraries(${PROJECT_NAME}
PUBLIC
${OpenCV_LIBS}
${EIGEN3_LIBS}
${CSPARSE_LIBRARY}
# for mac os (default)
#${PROJECT_SOURCE_DIR}/dependencies/g2o/lib/libg2o.dylib
# for linux (ubuntu)
${PROJECT_SOURCE_DIR}/dependencies/g2o/lib/libg2o.so
gtsam
${GFLAGS_LIBRARIES}
${GLOG_LIBRARIES}
)


# Build example

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/example)

add_executable(vdo_slam
example/vdo_slam.cc)
target_link_libraries(vdo_slam ${PROJECT_NAME})

##### Add some tests #####
add_executable(gtsam_basic_test
   test/gtsam_basic_test.cc
)

target_link_libraries(gtsam_basic_test
   gtsam
   ${GFLAGS_LIBRARIES}
   ${GLOG_LIBRARIES} )


