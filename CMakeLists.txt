cmake_minimum_required(VERSION 3.8)
project(shared_memory_tf)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# find dependencies
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    ament_index_cpp
    rclcpp
    tf2
    tf2_ros
    tf2_eigen
)

include_directories(
  SYSTEM include
)

add_executable(tf_writer
  src/tf_writer.cpp
)
target_include_directories(tf_writer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  eigen3
)
target_link_libraries(tf_writer
  Eigen3::Eigen
)
ament_target_dependencies(tf_writer
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

add_executable(tf_reader
  src/tf_reader.cpp
)
target_include_directories(tf_reader PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  eigen3
)
target_link_libraries(tf_reader
  Eigen3::Eigen
)
ament_target_dependencies(tf_reader
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Install all C++ targets
install(TARGETS
  tf_writer
  tf_reader
  DESTINATION lib/${PROJECT_NAME})

# Install include files
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
