#
# Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

cmake_minimum_required(VERSION 3.22)
project(aviary)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake_auto REQUIRED)

set(ament_dependencies
  "rclcpp"
  "rclcpp_components"
  "rosbag2_cpp"
  "audio_common_msgs")

foreach(pkg ${ament_dependencies})
  find_package(${pkg} REQUIRED)
endforeach()


#
# -------- bag_to_wave
#
add_executable(bag_to_wave  src/bag_to_wave.cpp)
ament_target_dependencies(bag_to_wave ${ament_dependencies})
target_include_directories(bag_to_wave PRIVATE include)
target_compile_features(bag_to_wave PRIVATE cxx_std_17)

install(PROGRAMS
  src/start_recording.py
  src/stop_recording.py
  DESTINATION lib/${PROJECT_NAME}/)

install(TARGETS
  bag_to_wave
  DESTINATION lib/${PROJECT_NAME}/)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.py")

if(BUILD_TESTING)
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)

  ament_copyright()
  ament_lint_cmake()
  ament_flake8()
  ament_pep257()
  ament_xmllint()
endif()

ament_package()
