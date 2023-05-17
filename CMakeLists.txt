cmake_minimum_required(VERSION 2.8.3)
project(pik_ros)

find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(PtscEigen REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra)

find_package( catkin REQUIRED COMPONENTS 
              roscpp rospy std_msgs 
              moveit_core
              moveit_ros_planning
              moveit_ros_planning_interface )

# pik_ros Library
set(LIBRARY_TARGET_NAME pik_ros)
set(LIBRARY_VERSION 0.0)

catkin_package( INCLUDE_DIRS include
                LIBRARIES ${LIBRARY_TARGET_NAME}
                DEPENDS
                EIGEN3 
                CATKIN_DEPENDS
                moveit_core
                moveit_ros_planning
                moveit_ros_planning_interface )

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

set(LIBRARY_HEADERS 
  include/PikRos.hpp 
  include/MoveItWrapper.hpp
)

add_library(${LIBRARY_TARGET_NAME}
  src/PikRos.cpp
  src/MoveItWrapper.cpp
)

target_link_libraries(${LIBRARY_TARGET_NAME}
  PRIVATE PtscEigen::PtscEigen
  ${catkin_LIBRARIES}
)

install(TARGETS ${LIBRARY_TARGET_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES ${LIBRARY_HEADERS}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)