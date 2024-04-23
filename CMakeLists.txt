cmake_minimum_required(VERSION 3.8)
project(shared_memory_tf)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# find dependencies
find_package(Eigen3 REQUIRED)

Include(FetchContent)

FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v2.13.10
)

FetchContent_MakeAvailable(Catch2)

include_directories(
  SYSTEM include
)

add_executable(tf_test
  tests/tf_test.cpp
)
target_include_directories(tf_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  eigen3
)
target_link_libraries(tf_test
  Eigen3::Eigen
  Catch2::Catch2
)

add_executable(tf_resetter
  src/tf_resetter.cpp
)
target_include_directories(tf_resetter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  eigen3
)
target_link_libraries(tf_resetter
  Eigen3::Eigen
)

# Install all C++ targets
install(TARGETS
  tf_test
  tf_resetter
  DESTINATION lib/${PROJECT_NAME})

# Install include files
install(
  DIRECTORY include/
  DESTINATION include
)

include(CTest)
#include(Catch)
#catch_discover_tests(tf_test)
