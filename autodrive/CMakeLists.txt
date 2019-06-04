cmake_minimum_required(VERSION 2.8.3)
project(autodrive)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

find_package(OpenCV REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  message_generation
  serial
  platform_rx_msg
)

add_message_files(FILES
  autodrive.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  INCLUDE_DIRS include
  LIBRARIES opencv
  serial
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  include ${OpenCV_INCLUDE_DIRS}
)


add_executable(autodrive_pub
  src/autodrive_pub.cpp
)
add_dependencies(autodrive_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(autodrive_pub
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(speed_debug
  src/speed_debug.cpp
)
add_dependencies(speed_debug ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(speed_debug
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(autodrive_sub
  src/autodrive_sub.cpp
)
add_dependencies(autodrive_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(autodrive_sub
  ${catkin_LIBRARIES}
)

#add_dependencies(your_executable ${catkin_EXPORTED_TARGETS})