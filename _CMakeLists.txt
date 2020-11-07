cmake_minimum_required(VERSION 2.8.3)
project(flight_scenario)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  genmsg
  message_generation
  flight_controller
  common_srv
)

# add_message_files(
#   FILES
#   PID_param.msg
#   Waypoint.msg
# )

add_service_files(
  FILES
  Flight_Command.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs message_runtime flight_controller common_srv
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)


FILE(GLOB SOURCE_FILES *.hpp *.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Navio2_cpp/*.cpp
        )

add_executable(${PROJECT_NAME} flight_scenario_node/main.cpp ${SOURCE_FILES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(example_node example_node/example_node.cpp ${SOURCE_FILES})
add_dependencies(example_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_node ${catkin_LIBRARIES})
