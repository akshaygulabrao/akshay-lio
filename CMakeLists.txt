cmake_minimum_required(VERSION 3.0.2)
project(akshay_lio)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    pcl_ros
)

find_package(PCL 1.8 REQUIRED)

catkin_package(CATKIN_DEPENDS roscpp)

include_directories()

add_executable(akshay_lio src/main.cpp)
target_link_libraries(akshay_lio ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_include_directories(akshay_lio PUBLIC ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})