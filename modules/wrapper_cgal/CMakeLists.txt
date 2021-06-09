cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 17)
project(test_bind)
message(${CMAKE_CURRENT_SOURCE_DIR})
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../pybind11 ${CMAKE_CURRENT_SOURCE_DIR}/pybin_bin EXCLUDE_FROM_ALL)

pybind11_add_module(mvbb main.cpp)
target_compile_definitions(mvbb PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})
target_link_libraries(mvbb PRIVATE tbb)
target_sources(mvbb PRIVATE Evaluator.cpp Evaluator.h helpers.h typedefs.h trim.h)

find_package(CGAL)
target_link_libraries(mvbb PRIVATE CGAL::CGAL)
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)
set(CMAKE_BUILD_TYPE Release)
#
