cmake_minimum_required(VERSION 3.5)
project(path_smoother)

add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED core highgui imgproc imgcodecs)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(
        include/
)

add_library(${PROJECT_NAME}
        include/basic_define/bucket_queue.h
        include/basic_define/math_utils.h
        include/basic_define/pose2d.h
        include/basic_define/vec2d.h
        include/basic_define/vec2i.h

        include/path_smoother/constants.h
        include/path_smoother/dynamic_voronoi.h
        include/path_smoother/path_smoother.h

        src/basic_define/bucket_queue.cpp
        src/basic_define/vec2d.cpp

        src/path_smoother/dynamic_voronoi.cpp
        src/path_smoother/path_smoother.cpp
        )

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

add_executable(path_smoother_example example/path_smoother_example.cpp)
target_link_libraries(path_smoother_example ${PROJECT_NAME} ${OpenCV_LIBS})

add_executable(generate_lane_curves example/generate_lane_curves.cpp)
target_link_libraries(generate_lane_curves ${PROJECT_NAME} ${OpenCV_LIBS})

