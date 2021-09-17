cmake_minimum_required(VERSION 3.5)

if(NOT TARGET spdlog)
    # Stand-alone build
    find_package(spdlog CONFIG REQUIRED)
endif()



# Setup targets
# 1.) python taget
pybind11_add_module(BoxOffice ../main.cpp)
file(GLOB sources LIST_DIRECTORIES true *.h *.cpp)
target_compile_definitions(BoxOffice PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})
target_link_libraries(BoxOffice PRIVATE tbb CGAL::CGAL CGAL::Eigen3_support spdlog::spdlog_header_only)
target_sources(BoxOffice PRIVATE ${sources})
set_property(TARGET BoxOffice PROPERTY POSITION_INDEPENDENT_CODE ON)
        #Evaluator.cpp Evaluator.h helpers.h helpers.cpp typedefs.h trim.h algorithm.cpp algorithm.h)

# 2.) test / lib target
add_library(${BINARY}_lib STATIC ${sources} BoxEntity.cpp BoxEntity.h)
target_link_libraries(${BINARY}_lib PRIVATE tbb CGAL::CGAL CGAL::Eigen3_support pybind11::module pybind11::lto pybind11::windows_extras spdlog::spdlog)
target_sources(${BINARY}_lib PRIVATE ${sources})


install(TARGETS BoxOffice DESTINATION ../../python)

#target_link_libraries(mvbb PRIVATE )
#set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)




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

