# Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
# Authors: Stefano Dafarra
#          Giulio Romualdi
# CopyPolicy: Released under the terms of the LGPLv2.1 or later

cmake_minimum_required(VERSION 3.16)
project(UnicyclePlannerTest)

include(FeatureSummary)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(YCM 0.12 REQUIRED)

include(GNUInstallDirs)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

if(MSVC)
  set(CMAKE_DEBUG_POSTFIX "d")
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

#include(AddInstallRPATHSupport)
#add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_FULL_BINDIR}"
#                          LIB_DIRS "${CMAKE_INSTALL_FULL_LIBDIR}"
#                          INSTALL_NAME_DIR "${CMAKE_INSTALL_FULL_LIBDIR}"
#                          USE_LINK_PATH)

if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
        message(STATUS "Setting build type to 'Release' as none was specified.")
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
    endif()
endif()

set_property(GLOBAL PROPERTY USE_FOLDERS 1)

include(AddUninstallTarget)

find_package(YARP 3.7.2 COMPONENTS os sig dev REQUIRED)
set(CMAKE_C_FLAGS "${YARP_C_FLAGS} ${CMAKE_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${YARP_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

find_package(ament_cmake REQUIRED)
find_package(Doxygen)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(test_msgs REQUIRED)

find_package(visualization_msgs REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

#add_subdirectory(doc)

#if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  ## the following line skips the linter which checks for copyrights
  ## uncomment the line when a copyright and license is not present in all source files
  ##set(ament_cmake_copyright_FOUND TRUE)
  ## the following line skips cpplint (only works in a git repo)
  ## uncomment the line when this package is not in a git repo
  ##set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
#endif()

ament_package()

feature_summary(WHAT ALL INCLUDE_QUIET_PACKAGES)

#cmake_minimum_required(VERSION 3.1)

#set (CMAKE_CXX_STANDARD 11)



#find_package(ament_cmake REQUIRED)
#set(rclcpp_DIR "/opt/ros/humble/share/rclcpp/cmake")
#set(ament_cmake_core_DIR "/opt/ros/humble/share/ament_cmake_core/cmake")
#set(ament_cmake_libraries_DIR "/opt/ros/humble/share/ament_cmake_libraries/cmake")
#set(ament_index_cpp_DIR "/opt/ros/humble/share/ament_index_cpp/cmake")
#include_directories(${rclcpp_DIR}, ${ament_cmake_core_DIR}, ${ament_cmake_libraries_DIR}, ${ament_index_cpp_DIR})
#include_directories(/opt/ros/humble/include/tf2_geometry_msgs)

#find_package(rclcpp REQUIRED)

#find_package(tf2_msgs REQUIRED)
#find_package(nav_msgs REQUIRED)

# UnicycleTest
add_executable(UnicycleTest UnicycleTest.cpp)
target_link_libraries(UnicycleTest UnicyclePlanner ${iDynTree_LIBRARIES})
add_test(NAME UnicycleTest
  COMMAND UnicycleTest)

# InterpolationTest
add_executable(InterpolationTest InterpolationTest.cpp)
target_link_libraries(InterpolationTest ${iDynTree_LIBRARIES} UnicyclePlanner)
add_test(NAME InterpolationTest
  COMMAND InterpolationTest)

# DcmInterpolationTest
add_executable(DcmInterpolationTest DcmInterpolationTest.cpp utils.cpp utils.h)
target_include_directories(DcmInterpolationTest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(DcmInterpolationTest ${iDynTree_LIBRARIES} UnicyclePlanner)
add_test(NAME DcmInterpolationTest
  COMMAND DcmInterpolationTest)

# ProtoCode
add_executable(ProtoCode ProtoCode.cpp)
target_link_libraries(ProtoCode UnicyclePlanner ${iDynTree_LIBRARIES} 
                      rclcpp::rclcpp 
                      tf2_ros::tf2_ros
                      tf2_geometry_msgs::tf2_geometry_msgs
                      tf2_msgs::tf2_msgs__rosidl_typesupport_cpp 
                      nav_msgs::nav_msgs__rosidl_typesupport_cpp
                      geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
                      visualization_msgs::visualization_msgs__rosidl_typesupport_cpp)
add_test(NAME ProtoCode
  COMMAND ProtoCode)
