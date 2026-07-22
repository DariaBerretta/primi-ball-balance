cmake_minimum_required(VERSION 3.19)

project(primi-ball-balance LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(YARP REQUIRED COMPONENTS os sig dev)
find_package(event-driven REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui imgcodecs)
find_package(Threads REQUIRED)

#============ Shared Image Preprocessing ============
add_library(image_preprocessing
    src/ImagePreprocessor.cpp
)

target_include_directories(image_preprocessing PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(image_preprocessing PUBLIC
    ${OpenCV_LIBS}
)

#============ Ball Detection ============
add_library(ball_detection
    src/BallDetector.cpp
)

target_include_directories(ball_detection PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(ball_detection PUBLIC
    image_preprocessing
    ${OpenCV_LIBS}
)

#============ Event Camera Reader ============
add_executable(eventCam_reader
    src/eventCam_reader.cpp
)

target_link_libraries(eventCam_reader PRIVATE
    YARP::YARP_os
    YARP::YARP_init
    ev::event-driven
    Threads::Threads
)

#============ RGB Camera Reader ============
add_executable(rgbCam_reader
    src/rgbCam_reader.cpp
)

target_link_libraries(rgbCam_reader PRIVATE
    image_preprocessing
    YARP::YARP_os
    YARP::YARP_sig
    YARP::YARP_init
    Threads::Threads
)

#============ Touch Reader ============
add_executable(touch_reader
    src/Touch_reader.cpp
)

target_link_libraries(touch_reader PRIVATE
    YARP::YARP_os
    YARP::YARP_sig
    YARP::YARP_init
    Threads::Threads
)

#============ Proprioception Reader ============
add_executable(proprio_reader
    src/Proprio_reader.cpp
)

target_link_libraries(proprio_reader PRIVATE
    YARP::YARP_os
    YARP::YARP_sig
    YARP::YARP_init
    YARP::YARP_dev
    Threads::Threads
)

#============ Ball Balance Controller ============
add_executable(Robot_control
    src/Robot_control.cpp
)

target_link_libraries(Robot_control PRIVATE
    YARP::YARP_os
    YARP::YARP_sig
    YARP::YARP_init
    YARP::YARP_dev
    Threads::Threads
)

#============ Image Preprocessor Test ============
add_executable(test_preprocessor
    src/test_preprocessor.cpp
)

target_link_libraries(test_preprocessor PRIVATE
    image_preprocessing
)

#============ Ball Detector Test ============
add_executable(test_ball_detector
    src/test_ball_detector.cpp
)

target_link_libraries(test_ball_detector PRIVATE
    ball_detection
)