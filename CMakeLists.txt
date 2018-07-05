cmake_minimum_required(VERSION 2.8.3)
project(stb_ros)

add_compile_options(-std=c++11)
#if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
#  set(CMAKE_BUILD_TYPE Release)
#endif()

find_package(catkin_simple REQUIRED)

catkin_python_setup()

catkin_simple()

# CYTHON MODULE
set(hapticsstb_rt_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/src/hapticsstb_rt)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${hapticsstb_rt_ROOT}/cmake)
include(UseCython)
add_custom_target(ReplicatePythonSourceTree ALL ${CMAKE_COMMAND} -P
  ${hapticsstb_rt_ROOT}/cmake/ReplicatePythonSourceTree.cmake
  ${CMAKE_CURRENT_BINARY_DIR}
  WORKING_DIRECTORY ${hapticsstb_rt_ROOT})

# if not set, hapticsstb_rt will build as standalone project
set(hapticsstb_rt_ROS TRUE)
include_directories(${hapticsstb_rt_ROOT}/include)
set_source_files_properties(${hapticsstb_rt_ROOT}/src/hapticsstb_rt.pyx PROPERTIES CYTHON_IS_CXX FALSE)
add_subdirectory(${hapticsstb_rt_ROOT}/src)

cs_install()

cs_install_scripts(scripts/runSTB.py)

cs_export()
