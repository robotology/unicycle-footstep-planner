# Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
# Authors: Stefano Dafarra
#          Giulio Romualdi
# CopyPolicy: Released under the terms of the LGPLv2.1 or later

cmake_minimum_required(VERSION 3.1)

set (CMAKE_CXX_STANDARD 11)

project(UnicyclePlannerTest)

# UnicycleTest
add_executable(UnicycleTest UnicycleTest.cpp)
target_link_libraries(UnicycleTest UnicyclePlanner ${iDynTree_LIBRARIES})
add_test(NAME UnicycleTest
  COMMAND UnicycleTest)

# InterpolationTest
add_executable(InterpolationTest interpolationTest.cpp)
target_link_libraries(InterpolationTest ${iDynTree_LIBRARIES} UnicyclePlanner)
add_test(NAME InterpolationTest
  COMMAND InterpolationTest)

# DcmInterpolationTest
add_executable(DcmInterpolationTest DcmInterpolationTest.cpp utils.cpp utils.h)
target_include_directories(DcmInterpolationTest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(DcmInterpolationTest ${iDynTree_LIBRARIES} UnicyclePlanner)
add_test(NAME DcmInterpolationTest
  COMMAND DcmInterpolationTest)
