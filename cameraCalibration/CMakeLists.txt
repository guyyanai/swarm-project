cmake_minimum_required(VERSION 3.16)
project(calibration)

set(CMAKE_CXX_STANDARD 17)
find_package(Threads REQUIRED)
find_package(nlohmann_json 3.2.0)

# Original OpenCV package (works only in Raspberry Pi)
find_package(OpenCV 3.0 QUIET)

# New OpenCV package (Installable on any Linux system)
# find_package(OpenCV 4.2 REQUIRED)
# Install new package using "sudo apt-get install libopencv-dev"

add_executable(cameraCalibration
        calibration.cpp)
target_link_libraries(cameraCalibration nlohmann_json::nlohmann_json ${OpenCV_LIBS} Threads::Threads)

add_executable(telloCalibration
        telloCalibration.cpp)
target_link_libraries(telloCalibration nlohmann_json::nlohmann_json ${OpenCV_LIBS} ctello Threads::Threads)
