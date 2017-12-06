cmake_minimum_required(VERSION 3.1)

set (CMAKE_CXX_STANDARD 11)


project(UnicyclePlannerTest)

add_executable(UnicycleTest UnicycleTest.cpp)

target_link_libraries(UnicycleTest UnicyclePlanner ${iDynTree_LIBRARIES})

add_executable(interpolationTest interpolationTest.cpp)

target_link_libraries(interpolationTest ${iDynTree_LIBRARIES} UnicyclePlanner)

add_test(NAME UnicycleTest
        COMMAND UnicycleTest)

add_test(NAME InterpolatorTest
    COMMAND InterpolatorTest)
