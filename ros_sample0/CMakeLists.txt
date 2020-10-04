cmake_minimum_required(VERSION 3.0.2)
project(ros_sample0)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  TimestampArray.msg
  TimestampEntry.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node1 src/node1.cpp)
target_link_libraries(${PROJECT_NAME}_node1 ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node1 ros_sample0_generate_messages_cpp)

add_executable(${PROJECT_NAME}_node2 src/node2.cpp)
target_link_libraries(${PROJECT_NAME}_node2 ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node2 ros_sample0_generate_messages_cpp)

add_executable(${PROJECT_NAME}_node3 src/node3.cpp)
target_link_libraries(${PROJECT_NAME}_node3 ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node3 ros_sample0_generate_messages_cpp)

