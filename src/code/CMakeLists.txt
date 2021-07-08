#cmake_minimum_required(VERSION 3.5)
#set(CMAKE_CXX_STANDARD 17)
#project(BoxOffice)
#set(BINARY ${CMAKE_PROJECT_NAME})
#
#message(..)
# Setup targets
# 1.) python taget
pybind11_add_module(${BINARY}_python ../main.cpp)
file(GLOB sources LIST_DIRECTORIES true *.h *.cpp)
target_compile_definitions(${BINARY}_python PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})
target_link_libraries(${BINARY}_python PRIVATE tbb CGAL::CGAL CGAL::Eigen3_support)
target_sources(${BINARY}_python PRIVATE ${sources})
        #Evaluator.cpp Evaluator.h helpers.h helpers.cpp typedefs.h trim.h algorithm.cpp algorithm.h)

# 2.) test / lib target
add_library(${BINARY}_lib STATIC ${sources})
target_link_libraries(${BINARY}_lib PRIVATE tbb CGAL::CGAL CGAL::Eigen3_support pybind11::module pybind11::lto pybind11::windows_extras)
target_sources(${BINARY}_lib PRIVATE ${sources})


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

