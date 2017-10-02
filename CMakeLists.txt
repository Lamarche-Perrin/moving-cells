cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project("Moving Cells")

set(CMAKE_CXX_FLAGS "-std=c++11")

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{HOME}/programming/lib/freenect2/lib/cmake/freenect2)
find_package(freenect2 REQUIRED)

set(OpenCV_DIR "/usr/lib/opencv")
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

include_directories(${OpenCV_INCLUDE_DIRS})

include_directories("/usr/include/libusb-1.0/")


INCLUDE_DIRECTORIES(
  ${freenect2_INCLUDE_DIR}
)

add_executable(moving-cells ./moving_cells.cpp)

target_link_libraries(moving-cells ${freenect2_LIBRARIES} ${OpenCV_LIBS})
