cmake_minimum_required(VERSION 2.8.3)
project(rosberrypi_cam)

find_package(catkin REQUIRED COMPONENTS
  image_transport roscpp cv_bridge camera_info_manager
)
find_package(raspicam REQUIRED)
find_package(OpenCV)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(rosberrypi_cam_node src/main.cpp)
target_link_libraries(rosberrypi_cam_node 
  ${raspicam_LIBS}
  ${raspicam_CV_LIBS}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})

install(TARGETS
  rosberrypi_cam_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
