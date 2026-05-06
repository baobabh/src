cmake_minimum_required(VERSION 3.10)
project(robust_chessboard_detect)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(robust_chessboard_detect robust_chessboard_detect.cpp)
target_link_libraries(robust_chessboard_detect ${OpenCV_LIBS})
