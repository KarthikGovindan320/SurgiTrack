cmake_minimum_required(VERSION 3.16)
project(SurgiTrack VERSION 1.0.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(OpenCV 4.5 REQUIRED COMPONENTS core imgproc calib3d objdetect highgui videoio)

set(SOURCES
    src/main.cpp
    src/instrument_detector.cpp
    src/instrument_tracker.cpp
    src/sterile_field_monitor.cpp
    src/or_visualizer.cpp
)
add_executable(surgitrack ${SOURCES})
target_include_directories(surgitrack PRIVATE src)
target_link_libraries(surgitrack PRIVATE ${OpenCV_LIBS})
target_compile_options(surgitrack PRIVATE -Wall -Wextra -O2)

