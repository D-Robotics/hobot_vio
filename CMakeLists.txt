# Copyright (c) 2024，D-Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(hobot_vio)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV 3.4.5 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# x3|j5|x86
set(PREFIX_PATH x3)
set(SYS_ROOT ${CMAKE_SYSROOT})

if(PLATFORM_X3)
  message("build platform X3")
  add_definitions(-DPLATFORM_X3)
  set(PREFIX_PATH x3)
elseif(PLATFORM_Rdkultra)
  set(PREFIX_PATH j5)
  add_definitions(-DPLATFORM_J5)
  message("build ${PROJECT_NAME} for platform Rdkultra")
  set(SYS_ROOT ${CMAKE_SYSROOT})
elseif(PLATFORM_X86)
  message("build platform x86")
  add_definitions(-DPLATFORM_X86)
  set(PREFIX_PATH x86)
  set(SYS_ROOT ${THIRD_PARTY})
else()
  if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
    message("invalid platform, build platform X3 default")
    add_definitions(-DPLATFORM_X3)
    set(PREFIX_PATH x3)
  elseif (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
    message("build platform X86")
    add_definitions(-DPLATFORM_X86)
    set(PREFIX_PATH x86)
    set(SYS_ROOT ${THIRD_PARTY})
  endif()
endif()

message("PREFIX_PATH is " ${PREFIX_PATH})
message("SYS_ROOT is " ${SYS_ROOT})

include_directories(include
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  ${PROJECT_SOURCE_DIR}/lib 
)

add_executable(${PROJECT_NAME}
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBRARIES}
        horizon_vio
        pthread
        dl)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  nav_msgs
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  cv_bridge
  tf2
  tf2_ros
)
        
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/config/
  DESTINATION lib/${PROJECT_NAME}/config/
)

install(DIRECTORY
        ${PROJECT_SOURCE_DIR}/lib/
        DESTINATION lib/
        )

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/launch/
  DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
