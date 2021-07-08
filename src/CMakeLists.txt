cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 17)
project(BoxOffice)
set(BINARY ${CMAKE_PROJECT_NAME})

message(..)

# Setup foreign projects
# pybind11 + Python Target
add_subdirectory(../modules/pybind11 ../pybin_bin EXCLUDE_FROM_ALL)

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

add_subdirectory(code)
option(PACKAGE_TESTS "Build the tests" ON)
if(PACKAGE_TESTS)
    add_subdirectory(tests)
endif()



#target_link_libraries(mvbb PRIVATE )
#set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)
#set(CMAKE_BUILD_TYPE Release)



#create_single_source_cgal_program("bench_obb.cpp")
#target_link_libraries(mvbb PUBLIC)

# Testing in C++


#find_package(Eigen3 3.1.0 REQUIRED) #(3.1.0 or greater)
#if (EIGEN3_FOUND)
##   include( ${EIGEN3_USE_FILE})
#   include(CGAL_Eigen_support)
#
#   message("Eigen was found ++")
#endif()
#create_single_source_cgal_program("obbsr.cpp")
#add_executable(tests obbsr.cpp)
#target_link_libraries(tests PUBLIC CGAL::Eigen_support)
#target_link_libraries(tests CGAL::CGAL)

