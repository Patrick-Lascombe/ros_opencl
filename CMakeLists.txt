cmake_minimum_required(VERSION 2.8.3)
project(ros_opencl)

add_compile_options(-std=c++11)

find_package(OpenCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sensor_msgs
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}
    CATKIN_DEPENDS
        roscpp
        roslib
        sensor_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCL_INCLUDE_DIRS}
    )

add_library(${PROJECT_NAME}
    src/${PROJECT_NAME}.cpp
)

target_link_libraries(${PROJECT_NAME} ${OpenCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(pointcloud_test1 tests/src/pointcloud_test1.cpp)
target_link_libraries(pointcloud_test1 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(pointcloud_test2 tests/src/pointcloud_test2.cpp)
target_link_libraries(pointcloud_test2 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(laserscan_test1 tests/src/laserscan_test1.cpp)
target_link_libraries(laserscan_test1 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(laserscan_test2 tests/src/laserscan_test2.cpp)
target_link_libraries(laserscan_test2 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(image_test1 tests/src/image_test1.cpp)
target_link_libraries(image_test1 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(image_test2 tests/src/image_test2.cpp)
target_link_libraries(image_test2 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(float_vector_test1 tests/src/float_vector_test1.cpp)
target_link_libraries(float_vector_test1 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(float_vector_test2 tests/src/float_vector_test2.cpp)
target_link_libraries(float_vector_test2 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(double_vector_test1 tests/src/double_vector_test1.cpp)
target_link_libraries(double_vector_test1 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(double_vector_test2 tests/src/double_vector_test2.cpp)
target_link_libraries(double_vector_test2 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(int_vector_test1 tests/src/int_vector_test1.cpp)
target_link_libraries(int_vector_test1 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(int_vector_test2 tests/src/int_vector_test2.cpp)
target_link_libraries(int_vector_test2 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(char_vector_test1 tests/src/char_vector_test1.cpp)
target_link_libraries(char_vector_test1 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(char_vector_test2 tests/src/char_vector_test2.cpp)
target_link_libraries(char_vector_test2 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(char_char_vectors_test1 tests/src/char_char_vectors_test1.cpp)
target_link_libraries(char_char_vectors_test1 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(char_char_vectors_test2 tests/src/char_char_vectors_test2.cpp)
target_link_libraries(char_char_vectors_test2 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(char_char_vectors_test3 tests/src/char_char_vectors_test3.cpp)
target_link_libraries(char_char_vectors_test3 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(char_int_vectors_test1 tests/src/char_int_vectors_test1.cpp)
target_link_libraries(char_int_vectors_test1 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(char_int_vectors_test2 tests/src/char_int_vectors_test2.cpp)
target_link_libraries(char_int_vectors_test2 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(char_int_vectors_test3 tests/src/char_int_vectors_test3.cpp)
target_link_libraries(char_int_vectors_test3 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(char_float_vectors_test1 tests/src/char_float_vectors_test1.cpp)
target_link_libraries(char_float_vectors_test1 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(char_float_vectors_test2 tests/src/char_float_vectors_test2.cpp)
target_link_libraries(char_float_vectors_test2 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(char_float_vectors_test3 tests/src/char_float_vectors_test3.cpp)
target_link_libraries(char_float_vectors_test3 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(char_double_vectors_test1 tests/src/char_double_vectors_test1.cpp)
target_link_libraries(char_double_vectors_test1 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(char_double_vectors_test2 tests/src/char_double_vectors_test2.cpp)
target_link_libraries(char_double_vectors_test2 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(char_double_vectors_test3 tests/src/char_double_vectors_test3.cpp)
target_link_libraries(char_double_vectors_test3 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(int_char_vectors_test1 tests/src/int_char_vectors_test1.cpp)
target_link_libraries(int_char_vectors_test1 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(int_char_vectors_test2 tests/src/int_char_vectors_test2.cpp)
target_link_libraries(int_char_vectors_test2 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(int_char_vectors_test3 tests/src/int_char_vectors_test3.cpp)
target_link_libraries(int_char_vectors_test3 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(int_int_vectors_test1 tests/src/int_int_vectors_test1.cpp)
target_link_libraries(int_int_vectors_test1 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(int_int_vectors_test2 tests/src/int_int_vectors_test2.cpp)
target_link_libraries(int_int_vectors_test2 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(int_int_vectors_test3 tests/src/int_int_vectors_test3.cpp)
target_link_libraries(int_int_vectors_test3 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(int_float_vectors_test1 tests/src/int_float_vectors_test1.cpp)
target_link_libraries(int_float_vectors_test1 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(int_float_vectors_test2 tests/src/int_float_vectors_test2.cpp)
target_link_libraries(int_float_vectors_test2 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(int_float_vectors_test3 tests/src/int_float_vectors_test3.cpp)
target_link_libraries(int_float_vectors_test3 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(int_double_vectors_test1 tests/src/int_double_vectors_test1.cpp)
target_link_libraries(int_double_vectors_test1 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(int_double_vectors_test2 tests/src/int_double_vectors_test2.cpp)
target_link_libraries(int_double_vectors_test2 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(int_double_vectors_test3 tests/src/int_double_vectors_test3.cpp)
target_link_libraries(int_double_vectors_test3 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(float_char_vectors_test1 tests/src/float_char_vectors_test1.cpp)
target_link_libraries(float_char_vectors_test1 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(float_char_vectors_test2 tests/src/float_char_vectors_test2.cpp)
target_link_libraries(float_char_vectors_test2 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(float_char_vectors_test3 tests/src/float_char_vectors_test3.cpp)
target_link_libraries(float_char_vectors_test3 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(float_float_vectors_test1 tests/src/float_float_vectors_test1.cpp)
target_link_libraries(float_float_vectors_test1 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(float_float_vectors_test2 tests/src/float_float_vectors_test2.cpp)
target_link_libraries(float_float_vectors_test2 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(float_float_vectors_test3 tests/src/float_float_vectors_test3.cpp)
target_link_libraries(float_float_vectors_test3 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(float_int_vectors_test1 tests/src/float_int_vectors_test1.cpp)
target_link_libraries(float_int_vectors_test1 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(float_int_vectors_test2 tests/src/float_int_vectors_test2.cpp)
target_link_libraries(float_int_vectors_test2 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(float_int_vectors_test3 tests/src/float_int_vectors_test3.cpp)
target_link_libraries(float_int_vectors_test3 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(float_double_vectors_test1 tests/src/float_double_vectors_test1.cpp)
target_link_libraries(float_double_vectors_test1 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(float_double_vectors_test2 tests/src/float_double_vectors_test2.cpp)
target_link_libraries(float_double_vectors_test2 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(float_double_vectors_test3 tests/src/float_double_vectors_test3.cpp)
target_link_libraries(float_double_vectors_test3 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(double_char_vectors_test1 tests/src/double_char_vectors_test1.cpp)
target_link_libraries(double_char_vectors_test1 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(double_char_vectors_test2 tests/src/double_char_vectors_test2.cpp)
target_link_libraries(double_char_vectors_test2 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(double_char_vectors_test3 tests/src/double_char_vectors_test3.cpp)
target_link_libraries(double_char_vectors_test3 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(double_float_vectors_test1 tests/src/double_float_vectors_test1.cpp)
target_link_libraries(double_float_vectors_test1 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(double_float_vectors_test2 tests/src/double_float_vectors_test2.cpp)
target_link_libraries(double_float_vectors_test2 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(double_float_vectors_test3 tests/src/double_float_vectors_test3.cpp)
target_link_libraries(double_float_vectors_test3 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(double_int_vectors_test1 tests/src/double_int_vectors_test1.cpp)
target_link_libraries(double_int_vectors_test1 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(double_int_vectors_test2 tests/src/double_int_vectors_test2.cpp)
target_link_libraries(double_int_vectors_test2 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(double_int_vectors_test3 tests/src/double_int_vectors_test3.cpp)
target_link_libraries(double_int_vectors_test3 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(double_double_vectors_test1 tests/src/double_double_vectors_test1.cpp)
target_link_libraries(double_double_vectors_test1 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(double_double_vectors_test2 tests/src/double_double_vectors_test2.cpp)
target_link_libraries(double_double_vectors_test2 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(double_double_vectors_test3 tests/src/double_double_vectors_test3.cpp)
target_link_libraries(double_double_vectors_test3 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(int_vector_buffer_size_test1 tests/src/int_vector_buffer_size_test1.cpp)
target_link_libraries(int_vector_buffer_size_test1 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(int_vector_buffer_size_test2 tests/src/int_vector_buffer_size_test2.cpp)
target_link_libraries(int_vector_buffer_size_test2 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(char_char_char_vectors_test1 tests/src/char_char_char_vectors_test1.cpp)
target_link_libraries(char_char_char_vectors_test1 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(char_char_char_vectors_test2 tests/src/char_char_char_vectors_test2.cpp)
target_link_libraries(char_char_char_vectors_test2 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(char_char_char_vectors_test3 tests/src/char_char_char_vectors_test3.cpp)
target_link_libraries(char_char_char_vectors_test3 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(char_char_char_vectors_test4 tests/src/char_char_char_vectors_test4.cpp)
target_link_libraries(char_char_char_vectors_test4 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(pointcloud_revert_cpu tests/src/pointcloud_revert_cpu.cpp)
target_link_libraries(pointcloud_revert_cpu ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(pointcloud_revert_gpu tests/src/pointcloud_revert_gpu.cpp)
target_link_libraries(pointcloud_revert_gpu ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(test_minmax_pc tests/src/test_minmax_pc.cpp)
target_link_libraries(test_minmax_pc ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(pc_cropbox tests/src/pc_cropbox.cpp)
target_link_libraries(pc_cropbox ${PROJECT_NAME} ${catkin_LIBRARIES})
