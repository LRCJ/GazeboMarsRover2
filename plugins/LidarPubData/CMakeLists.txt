cmake_minimum_required(VERSION 2.8.3)
project(LidarPubData_plugins)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  gazebo_ros
)
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

#catkin_package(
#  INCLUDE_DIRS include ${GAZEBO_INCLUDE_DIRS}
#  LIBRARIES gazebo_ros_velodyne_laser gazebo_ros_velodyne_gpu_laser
#  CATKIN_DEPENDS roscpp sensor_msgs gazebo_ros
#)

include_directories(
  inc
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME} src/LidarPubData.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  RayPlugin #必须要加上
)

#add_library(LidarPubDataGpu src/LidarPubData.cpp)
#target_link_libraries(LidarPubDataGpu
#  ${catkin_LIBRARIES}
#  ${GAZEBO_LIBRARIES}
#  GpuRayPlugin
#)
#target_compile_definitions(LidarPubDataGpu PRIVATE GAZEBO_GPU_RAY=1)

#install(TARGETS gazebo_ros_velodyne_laser gazebo_ros_velodyne_gpu_laser
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)

#install(DIRECTORY include/${PROJECT_NAME}/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)
