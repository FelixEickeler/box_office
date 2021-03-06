cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 17)
project(BoxOffice)
set(BINARY ${CMAKE_PROJECT_NAME})

option(EIGEN_NO_DEBUG "Disables Eigen's assertions" OFF)

message("Current Directory:" ${PROJECT_SOURCE_DIR})
#set(CMAKE_CXX_FLAGS_DEBUG "-g -O2")
#set(CMAKE_BUILD_TYPE "Release")

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/_build/ CACHE PATH "..." FORCE)
endif()

add_compile_definitions (CGAL_DISABLE_ROUNDING_MATH_CHECK=ON)
# Setup foreign projects
# pybind11 + Python Target
MESSAGE(${PROJECT_SOURCE_DIR})
add_subdirectory(../modules/pybind11 ./pybind_bin EXCLUDE_FROM_ALL)

# CGal
set(CGAL_EIGEN3_ENABLED TRUE)
find_package(CGAL REQUIRED)
include(${CGAL_USE_FILE})

# Eigen 3
find_package(Eigen3 3.1.0 REQUIRED) #(3.1.0 or greater)
include(CGAL_Eigen3_support)
if(NOT TARGET CGAL::Eigen3_support)
    message(
            STATUS "This project requires the Eigen library, and will not be compiled.")
    return()
endif()

# SPDLOG
add_subdirectory(../modules/spdlog ./spdlog EXCLUDE_FROM_ALL)
# Boost
#find_package( Boost REQUIRED COMPONENTS ranges)
#include_directories( ${Boost_INCLUDE_DIRS} )

add_subdirectory(code)
option(PACKAGE_TESTS "Build the tests" ON)
if(PACKAGE_TESTS)
    add_subdirectory(tests)
endif()


#target_link_libraries(mvbb PRIVATE )
#set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)

#Fix for CGAL
set(CMAKE_BUILD_TYPE Release)




#create_single_source_cgal_program("bench_obb.cpp")
#target_link_libraries(mvbb PUBLIC)

