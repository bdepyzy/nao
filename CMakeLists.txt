cmake_minimum_required(VERSION 3.0)
project(nao_slam CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# Check C++14 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
if(COMPILER_SUPPORTS_CXX14)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
   add_definitions(-DCOMPILEDWITHC11)
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support.")
endif()

# ORB_SLAM3 paths
set(ORB_SLAM3_ROOT /dev/data/build/ORB_SLAM3)

# Find Eigen3
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

# Include directories
include_directories(
    ${ORB_SLAM3_ROOT}
    ${ORB_SLAM3_ROOT}/include
    ${ORB_SLAM3_ROOT}/include/CameraModels
    ${ORB_SLAM3_ROOT}/Thirdparty/Sophus
    ${EIGEN3_INCLUDE_DIR}
    /usr/include/eigen3
    /usr/local/include/eigen3
    /usr/local/include/opencv4
    ${Pangolin_INCLUDE_DIRS}
)

# OpenCV
find_package(OpenCV REQUIRED)

# Libraries
set(LIBS
    ${OpenCV_LIBS}
    ${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so
    ${Pangolin_LIBRARIES}
    GL
    GLU
    pthread
)

# Mono NAO executable
add_executable(mono_nao mono_nao.cc)
target_link_libraries(mono_nao ${LIBS})

# Mono-inertial NAO executable (with IMU)
add_executable(mono_inertial_nao mono_inertial_nao.cc)
target_link_libraries(mono_inertial_nao ${LIBS})

# Mono with robot control (WASDQE)
add_executable(mono_nao_control mono_nao_control.cc)
target_link_libraries(mono_nao_control ${LIBS})
