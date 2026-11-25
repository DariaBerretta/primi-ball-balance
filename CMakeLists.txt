cmake_minimum_required(VERSION 3.19)
project(primi-ball-balance LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Dependencies shared by the sensor readers, estimators, and optional tools.
find_package(YARP REQUIRED COMPONENTS os sig dev)
find_package(event-driven REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui imgcodecs)
find_package(Threads REQUIRED)
find_package(hpe-core REQUIRED)
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)

# RGB preprocessing and colour-based ball detection.
add_library(image_preprocessing src/ImagePreprocessor.cpp)
target_include_directories(image_preprocessing PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(image_preprocessing PUBLIC ${OpenCV_LIBS})

add_library(ball_detection src/BallDetector.cpp)
target_include_directories(ball_detection PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(ball_detection PUBLIC image_preprocessing ${OpenCV_LIBS})

# Camera readers. Event relaying avoids duplicate robot-to-laptop streams.
add_executable(event_relay src/event_camera_relay.cpp)
target_link_libraries(event_relay PRIVATE YARP::YARP_os YARP::YARP_init ev::event-driven Threads::Threads)

add_executable(rgb_processor src/rgb_camera_processor.cpp)
target_link_libraries(rgb_processor PRIVATE ball_detection YARP::YARP_os YARP::YARP_sig YARP::YARP_init Threads::Threads)

add_executable(bma400_reader src/bma400_reader.cpp)
target_link_libraries(bma400_reader PRIVATE YARP::YARP_os YARP::YARP_init)

# Standalone right-arm trajectory playback (never auto-started).
add_executable(Robot_control src/Robot_control.cpp)
target_link_libraries(Robot_control PRIVATE YARP::YARP_os YARP::YARP_sig YARP::YARP_init YARP::YARP_dev Threads::Threads)

# Optional event-based pose estimation and its Qt event-camera viewer.
add_executable(moveEnetOFK_online src/moveEnetOFK_online.cpp)
target_link_libraries(moveEnetOFK_online PRIVATE
    YARP::YARP_os YARP::YARP_sig YARP::YARP_init
    ev::event-driven hpe-core::hpe-core ${OpenCV_LIBS} Threads::Threads)

add_executable(event_control_gui src/event_control_gui.cpp)
target_link_libraries(event_control_gui PRIVATE
    Qt${QT_VERSION_MAJOR}::Widgets YARP::YARP_os YARP::YARP_init
    ev::event-driven ${OpenCV_LIBS} Threads::Threads)

# Standalone camera controller source (also built on icub-ultrascale).
add_subdirectory(tools/camera_control)
