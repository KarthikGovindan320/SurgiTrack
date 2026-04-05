cmake_minimum_required(VERSION 3.16)
project(MediMarkAR VERSION 1.0.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(OpenCV 4.5 REQUIRED COMPONENTS core imgproc calib3d objdetect highgui videoio)
set(SOURCES
    src/main.cpp
    src/aruco_detector.cpp
    src/kalman_predictor.cpp
    src/collision_checker.cpp
    src/visualizer.cpp
)
add_executable(medimark_ar ${SOURCES})
target_include_directories(medimark_ar PRIVATE src)
target_link_libraries(medimark_ar PRIVATE ${OpenCV_LIBS})
target_compile_options(medimark_ar PRIVATE -Wall -Wextra -O2)
