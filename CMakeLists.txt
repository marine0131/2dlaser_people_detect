cmake_minimum_required(VERSION 2.8.3)
project(laser_people_detect)

# look for bfl (Bayesian Filtering Library)
# find_package(PkgConfig REQUIRED)
# pkg_check_modules(BFL REQUIRED orocos-bfl)

# include_directories(${BFL_INCLUDE_DIRS})
# link_directories(${BFL_LIBRARY_DIRS})

# Look for Bullet
# pkg_check_modules(BULLET bullet)
# include_directories(${BULLET_INCLUDE_DIRS})

## Find pcl
find_package(PCL 1.7 REQUIRED)

## find opencv
find_package(OpenCV REQUIRED)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  std_srvs
  sensor_msgs
  laser_geometry
  tf
  visualization_msgs
  rosbag
  dynamic_reconfigure
)

## Specify additional locations of header files
include_directories(
  include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
)

catkin_package(INCLUDE_DIRS include
  CATKIN_DEPENDS sensor_msgs std_msgs geometry_msgs visualization_msgs)


add_executable(train_body_detector 
               src/train_body_detector.cpp
               src/calc_people_features.cpp)

target_link_libraries(train_body_detector
    ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${OpenCV_LIBS} ${PCL_FILTERS_LIBRARIES}
)

## Add cmake target dependencies of the executable/library
# add_dependencies(leg_detector people_msgs_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})
# add_dependencies(train_leg_detector ${PROJECT_NAME}_gencpp)
# add_dependencies(train_leg_detector ${PROJECT_NAME}_gencfg)
