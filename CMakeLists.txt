cmake_minimum_required(VERSION 2.8.3)
project(hear_navio_mc_example)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  hear_hardware_interface
  hear_architecture
  hear_ros_bridge
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    hear_ros_bridge
    roscpp
  DEPENDS
    system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(mc_example_node example_node/example_node.cpp)
add_executable(mc_slam_mrft_node example_node/SLAM_tuning_node.cpp)
add_executable(mc_translation_node example_node/traslation_node.cpp)
add_executable(mc_test_node example_node/test_node.cpp)


target_link_libraries(mc_example_node ${catkin_LIBRARIES})
target_link_libraries(mc_test_node ${catkin_LIBRARIES})
target_link_libraries(mc_slam_mrft_node ${catkin_LIBRARIES})
target_link_libraries(mc_translation_node ${catkin_LIBRARIES})
