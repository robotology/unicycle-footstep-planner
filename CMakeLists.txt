cmake_minimum_required(VERSION 3.0)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Enable C++11
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED 11)

# include macros for warnings
include(AddWarningsConfigurationToTargets)

include(CMakePackageConfigHelpers)


project(UnicyclePlanner CXX)

option(ENABLE_RPATH "Enable RPATH for this library" ON)
mark_as_advanced(ENABLE_RPATH)
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
                          LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
                          DEPENDS ENABLE_RPATH
                          USE_LINK_PATH)

find_package(iDynTree 0.9.2 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

set(UCPLANNER_SOURCES src/ControlledUnicycle.cpp
                      src/UnicyclePlanner.cpp
                      src/UnicycleController.cpp
                      src/UnicycleOptimization.cpp
                      src/UnicycleFoot.cpp
                      src/FootPrint.cpp
                      src/FootPrintsInterpolator.cpp
                      src/UnicycleTrajectoryGenerator.cpp)

set(UCPLANNER_HEADERS include/ControlledUnicycle.h
                      include/UnicycleController.h
                      include/UnicycleOptimization.h
                      include/UnicycleFoot.h
                      include/FootPrint.h
                      include/UnicyclePlanner.h
                      include/FootPrintsInterpolator.h
                      include/UnicycleTrajectoryGenerator.h)

add_library(UnicyclePlanner  SHARED ${UCPLANNER_HEADERS} ${UCPLANNER_SOURCES})
target_include_directories(UnicyclePlanner PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_include_directories(UnicyclePlanner PUBLIC $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(UnicyclePlanner ${iDynTree_LIBRARIES})

set_property(TARGET UnicyclePlanner PROPERTY PUBLIC_HEADER ${UCPLANNER_HEADERS})


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

install(FILES ${CMAKE_BINARY_DIR}/UnicyclePlannerConfig.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/UnicyclePlanner)

include(AddUninstallTarget)

include(CTest)
if(BUILD_TESTING)
   add_subdirectory(tests)
endif()

