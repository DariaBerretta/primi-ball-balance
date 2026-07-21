cmake_minimum_required(VERSION 3.19)

project(primi-ball-balance LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(YARP REQUIRED COMPONENTS os sig dev)
find_package(event-driven REQUIRED)
find_package(Threads REQUIRED)

#============ Event Camera Reader ============
add_executable(eventCam_reader
    src/eventCam_reader.cpp)

target_link_libraries(eventCam_reader PRIVATE
    YARP::YARP_os
    YARP::YARP_init
    ev::event-driven
    Threads::Threads)


#============ RGB Camera Reader ============
add_executable(rgbCam_reader 
    src/rgbCam_reader.cpp)

target_link_libraries(rgbCam_reader PRIVATE
    YARP::YARP_os
    YARP::YARP_sig
    YARP::YARP_init
    Threads::Threads)

#============ Touch Reader ============
add_executable(touch_reader
    src/Touch_reader.cpp)

target_link_libraries(touch_reader PRIVATE
    YARP::YARP_os
    YARP::YARP_sig
    YARP::YARP_init
    Threads::Threads)

#============ Proprioception Reader ============
add_executable(proprio_reader
    src/Proprio_reader.cpp)

target_link_libraries(proprio_reader PRIVATE
    YARP::YARP_os
    YARP::YARP_sig
    YARP::YARP_init
    YARP::YARP_dev
    Threads::Threads)

#============ Ball Balance Controller ============
add_executable(Robot_control
    src/Robot_control.cpp)

target_link_libraries(Robot_control PRIVATE
    YARP::YARP_os
    YARP::YARP_sig
    YARP::YARP_init
    YARP::YARP_dev
    Threads::Threads)