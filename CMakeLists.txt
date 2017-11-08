cmake_minimum_required (VERSION 2.8 FATAL_ERROR)

project ("Moving Cells")

set (CMAKE_CXX_FLAGS "-std=c++11")
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set (CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{HOME}/programming/lib/freenect2/lib/cmake/freenect2)
set (OpenCV_DIR "/usr/lib/opencv")

find_package (OpenCV REQUIRED COMPONENTS core imgproc highgui)
find_package (freenect2 REQUIRED)
find_package (Threads)

include_directories (${OpenCV_INCLUDE_DIRS})
include_directories (${freenect2_INCLUDE_DIR})
include_directories ("/usr/include/libusb-1.0/")

add_executable (moving-cells ./src/moving_cells.cpp)
add_executable (time-delays ./src/time_delays.cpp)
add_executable (time-ghosts ./src/time_ghosts.cpp)
add_executable (webcam-delays ./src/webcam_delays.cpp)

target_link_libraries (moving-cells ${freenect2_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries (time-delays ${freenect2_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries (time-ghosts ${freenect2_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries (webcam-delays ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS})
