cmake_minimum_required(VERSION 3.22)
project(2022summerCourse)

set(CMAKE_CXX_STANDARD 17)
find_package(Threads REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)

# Original OpenCV package (works only in Raspberry Pi)
find_package(OpenCV 3.0 QUIET)

# New OpenCV package (Installable on any Linux system)
# find_package(OpenCV 4.2 REQUIRED)
# Install new package using "sudo apt-get install libopencv-dev"

include_directories(include)
add_library(aruco src/aruco.cpp)
add_library(controller src/controller.cpp)
add_library(tellocom src/tellocom.cpp)
target_link_libraries(aruco -lpthread ${OpenCV_LIBS})
add_executable(runAruco
        runAruco.cpp)
target_link_libraries(runAruco aruco controller tellocom nlohmann_json::nlohmann_json ctello)
