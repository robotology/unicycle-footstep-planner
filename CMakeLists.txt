cmake_minimum_required(VERSION 3.1)

# ouptut paths
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

# Build shared libs
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
endif()

option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Enable C++14
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# include macros for warnings
include(AddWarningsConfigurationToTargets)

include(CMakePackageConfigHelpers)

project(UnicyclePlanner
        LANGUAGES CXX
        VERSION 0.1.102)

option(ENABLE_RPATH "Enable RPATH for this library" ON)
mark_as_advanced(ENABLE_RPATH)
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
                          LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
                          DEPENDS ENABLE_RPATH
                          USE_LINK_PATH)

# Encourage user to specify a build type (e.g. Release, Debug, etc.), otherwise set it to Release.
if(NOT CMAKE_CONFIGURATION_TYPES)
  if(NOT CMAKE_BUILD_TYPE)
      message(STATUS "Setting build type to 'Release' as none was specified.")
      set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
  endif()
endif()

find_package(iDynTree 0.11.103 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

set(UCPLANNER_SOURCES src/ControlledUnicycle.cpp
                      src/UnicyclePlanner.cpp
                      src/UnicycleController.cpp
                      src/UnicycleOptimization.cpp
                      src/UnicycleFoot.cpp
                      src/FootPrint.cpp
                      src/FootPrintsInterpolator.cpp
                      src/UnicycleTrajectoryGenerator.cpp
                      src/FeetGenerator.cpp
                      src/FeetCubicSplineGenerator.cpp
                      src/UnicycleGenerator.cpp
                      src/ZMPTrajectoryGenerator.cpp
                      src/CoMHeightTrajectoryGenerator.cpp
                      src/DCMTrajectoryGenerator.cpp
                      src/DCMTrajectoryGeneratorHelper.cpp
                      src/FeetMinimumJerkGenerator.cpp)

set(UCPLANNER_HEADERS include/ControlledUnicycle.h
                      include/UnicycleController.h
                      include/UnicycleOptimization.h
                      include/UnicycleFoot.h
                      include/FootPrint.h
                      include/UnicyclePlanner.h
                      include/FootPrintsInterpolator.h
                      include/UnicycleTrajectoryGenerator.h
                      include/FeetGenerator.h
                      include/FeetCubicSplineGenerator.h
                      include/UnicycleGenerator.h
                      include/ZMPTrajectoryGenerator.h
                      include/CoMHeightTrajectoryGenerator.h
                      include/StepPhase.h
                      include/DCMTrajectoryGenerator.h
                      include/DCMTrajectoryGeneratorHelper.h
                      include/FeetMinimumJerkGenerator.h)

add_library(UnicyclePlanner ${UCPLANNER_HEADERS} ${UCPLANNER_SOURCES})
target_include_directories(UnicyclePlanner PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_include_directories(UnicyclePlanner PUBLIC $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(UnicyclePlanner ${iDynTree_LIBRARIES})

set_property(TARGET UnicyclePlanner PROPERTY PUBLIC_HEADER ${UCPLANNER_HEADERS})
target_compile_features(UnicyclePlanner PUBLIC cxx_attribute_deprecated)

# install
install(TARGETS UnicyclePlanner
        EXPORT unicycle_planner
        COMPONENT runtime
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT unicycle_planner
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/UnicyclePlanner)

configure_package_config_file(${CMAKE_SOURCE_DIR}/cmake/unicyclePlannerConfig.cmake.in
                              ${CMAKE_BINARY_DIR}/UnicyclePlannerConfig.cmake
                              INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/UnicyclePlanner)

write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/UnicyclePlannerConfigVersion.cmake  VERSION ${UnicyclePlanner_VERSION} COMPATIBILITY SameMajorVersion )

install(FILES ${CMAKE_BINARY_DIR}/UnicyclePlannerConfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/UnicyclePlannerConfigVersion.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/UnicyclePlanner)

include(AddUninstallTarget)

include(CTest)
if(BUILD_TESTING)
   add_subdirectory(tests)
endif()
