cmake_minimum_required(VERSION 3.0.2)
project(akshay_lio)

find_package(catkin REQUIRED COMPONENTS
roscpp)

catkin_package(CATKIN_DEPENDS roscpp)

include_directories()

add_executable(akshay_lio src/main.cpp)
target_link_libraries(akshay_lio ${catkin_LIBRARIES})
target_include_directories(akshay_lio PUBLIC ${catkin_INCLUDE_DIRS})