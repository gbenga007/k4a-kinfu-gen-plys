cmake_minimum_required(VERSION 2.8)
project(kinfu_create_plys)
find_package(OpenCV REQUIRED)
find_package(k4a REQUIRED)
find_package(k4arecord REQUIRED)

add_executable(kinfu_create_plys  grab_images_k4a.cpp)
target_link_libraries(kinfu_create_plys ${OpenCV_LIBS} k4a::k4arecord k4a::k4a)

