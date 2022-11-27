cmake_minimum_required(VERSION 2.8)
project(ObjSLAM)

add_definitions(-std=c++17 -Wall -Wextra)


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

# # Check C++11 or C++0x support
# include(CheckCXXCompilerFlag)
# CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
# CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
# if(COMPILER_SUPPORTS_CXX11)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#    add_definitions(-DCOMPILEDWITHC11)
#    message(STATUS "Using flag -std=c++11.")
# elseif(COMPILER_SUPPORTS_CXX0X)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#    add_definitions(-DCOMPILEDWITHC0X)
#    message(STATUS "Using flag -std=c++0x.")
# else()
#    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
# endif()

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
find_package(matplotlib_cpp REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include

${EIGEN3_INCLUDE_DIR}
${CSPARSE_INCLUDE_DIR}
${GLOG_INCLUDE_DIRS} 
${GTSAM_INCLUDE_DIR}
   /usr/include/python2.7
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/DataProvider.cc
src/UtilsGtsam.cc
src/FeatureTracker.cc

src/utils/ParamParser.cc
src/utils/UtilsOpenCV.cc
src/utils/Logger.cc

src/viz/Display.cc
src/viz/DisplayParams.cc
src/viz/OpenCvDisplay.cc
src/viz/OpenCvVisualizer3D.cc
src/viz/Visualizer.cc


src/Camera.cc
src/ORBextractor.cc
src/Frame.cc
src/Optimizer.cc

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
gtsam_unstable
${GFLAGS_LIBRARIES}
${GLOG_LIBRARIES}
python2.7 #for matplotlib_cpp
)


# Build example

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/example)

add_executable(vdo_slam
example/vdo_slam.cc)
target_link_libraries(vdo_slam ${PROJECT_NAME})

############################### TESTS ##########################################
### Add testing
option(VDO_SLAM_BUILD_TESTS "Build tests" ON)
if(VDO_SLAM_BUILD_TESTS)
  # Download and unpack googletest at configure time
  # TODO Consider doing the same for glog, gflags, although it might
  # make the command `cmake ..` a bit slow.
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/gtest.cmake
    external/googletest-download/CMakeLists.txt)
  execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
      WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/external/googletest-download"
      OUTPUT_QUIET)
  execute_process(COMMAND "${CMAKE_COMMAND}" --build .
      WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/external/googletest-download"
      OUTPUT_QUIET)

  # Prevent GoogleTest from overriding our compiler/linker options
  # when building with Visual Studio
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

  # Add googletest directly to our build. This adds
  # the following targets: gtest, gtest_main, gmock
  # and gmock_main
  # Exclude these targets from installation with EXCLUDE_FROM_ALL
  # Check issue: https://github.com/google/googletest/issues/868
  add_subdirectory("${CMAKE_BINARY_DIR}/external/googletest-src"
                   "${CMAKE_BINARY_DIR}/external/googletest-build"
                   EXCLUDE_FROM_ALL)

  # The gtest/gmock targets carry header search path
  # dependencies automatically when using CMake 2.8.11 or
  # later. Otherwise we have to add them here ourselves.
  if(CMAKE_VERSION VERSION_LESS 2.8.11)
      include_directories("${gtest_SOURCE_DIR}/include"
                          "${gmock_SOURCE_DIR}/include")
  endif()

  # Generate gtests.
  include(CTest)
  add_executable(testVdoSlam
    test/testCameraParam.cc
    test/testVdoSlam.cc
    )
  target_link_libraries(testVdoSlam gtest ${PROJECT_NAME})

  if(CMAKE_VERSION VERSION_LESS 3.10)
    enable_testing()
    include(FindGTest)
    gtest_add_tests(testVdoSlam "" AUTO)
  else()
    include(GoogleTest)
    gtest_discover_tests(testVdoSlam)
  endif()
endif(VDO_SLAM_BUILD_TESTS)