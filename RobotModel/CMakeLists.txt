cmake_minimum_required(VERSION 2.8.3)
project(robotmodel)
set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories("./include")
include_directories("./")
include_directories("../")
include_directories("../Common/include")
include_directories("/usr/include/eigen3")

file(GLOB_RECURSE sources "src/*.cpp")  

add_library(robotmodel SHARED ${sources})