cmake_minimum_required(VERSION 3.19)

project(primi-ball-balance LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(YARP REQUIRED COMPONENTS os sig)
find_package(event-driven REQUIRED)
find_package(Threads REQUIRED)

add_executable(eventCam_reader
    src/eventCam_reader.cpp)

target_link_libraries(eventCam_reader PRIVATE
    YARP::YARP_os
    YARP::YARP_init
    ev::event-driven
    Threads::Threads)

add_executable(rgbCam_reader 
    src/rgbCam_reader.cpp)

target_link_libraries(rgbCam_reader PRIVATE
    YARP::YARP_os
    YARP::YARP_sig
    YARP::YARP_init
    Threads::Threads)
