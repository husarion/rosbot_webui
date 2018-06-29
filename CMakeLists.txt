cmake_minimum_required(VERSION 2.8.3)
project(rosbot_webui)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp tf
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pose_to_tf_transform src/pose_to_tf.cpp)

target_link_libraries(pose_to_tf_transform
  ${catkin_LIBRARIES}
)

add_executable(tf_to_pose src/tf_to_pose.cpp)

target_link_libraries(tf_to_pose
  ${catkin_LIBRARIES}
)
