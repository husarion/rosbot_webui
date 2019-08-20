cmake_minimum_required(VERSION 2.8.3)
project(rosbot_webui)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp tf image_transport cv_bridge
)
find_package(OpenCV REQUIRED)

catkin_package(
)

include_directories(
  include
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

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY conf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY edit
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(TARGETS pose_to_tf_transform tf_to_pose
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
