cmake_minimum_required(VERSION 3.8)
project(romea_path_following)

if(NOT CMAKE_CXX_STANDARD)
 set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp
    nodelet
    sensor_msgs

    romea_common_utils
    romea_path_msgs
    romea_path_utils
    romea_joy
    romea_mobile_base_utils
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

# find_package(romea_mobile_base_utils REQUIRED)
# find_package(romea_mobile_base_msgs REQUIRED)
# find_package(romea_joystick_utils REQUIRED)
# find_package(romea_path_utils REQUIRED)
# find_package(romea_path_msgs REQUIRED)
find_package(romea_core_common REQUIRED)
find_package(romea_core_mobile_base REQUIRED)
find_package(romea_core_control REQUIRED)
find_package(romea_core_path REQUIRED)
find_package(romea_core_path_following REQUIRED)
# find_package(romea_following_utils REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${romea_core_control_INCLUDE_DIRS}
  ${romea_core_path_following_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/path_following_nodelet.cpp
  # src/path_following.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  romea_core_path_following::romea_core_path_following
  ${catkin_LIBRARIES}
)

# if(BUILD_TESTING)
#     add_subdirectory(test)
# endif()
