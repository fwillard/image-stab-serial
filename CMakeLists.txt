cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(ImageStab VERSION 1.0)

find_package(OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

# require C++ 11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_subdirectory(src)
