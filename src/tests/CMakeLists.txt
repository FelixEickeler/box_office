set(BINARY ${CMAKE_PROJECT_NAME}_testing)
if(NOT TARGET spdlog)
    # Stand-alone build
    find_package(spdlog REQUIRED)
endif()

# Setup targets

#setup gtest
add_subdirectory("${PROJECT_SOURCE_DIR}/../modules/googletest" "modules/googletest" EXCLUDE_FROM_ALL)
mark_as_advanced(
        BUILD_GMOCK BUILD_GTEST BUILD_SHARED_LIBS
        gmock_build_tests gtest_build_samples gtest_build_tests
        gtest_disable_pthreads gtest_force_shared_crt gtest_hide_internal_symbols
)

enable_testing()
include(GoogleTest)

# setup test targets
file(GLOB_RECURSE TEST_SOURCES LIST_DIRECTORIES false *.h *.cpp)

# Single experimental target
if(BOX_OFFICE_VALGRIND_TARGET)
    add_executable(valgrind_decompose valgrind_decompose_target.cpp )
    target_link_libraries(valgrind_decompose PRIVATE CGAL::CGAL CGAL::Eigen3_support spdlog::spdlog)
    target_include_directories(valgrind_decompose PUBLIC ../code)
    target_link_libraries(valgrind_decompose PUBLIC ${CMAKE_PROJECT_NAME}_lib )
    #pybind11::module pybind11::lto pybind11::windows_extras
endif()

# actual test targets for gtest
list(REMOVE_ITEM TEST_SOURCES "${PROJECT_SOURCE_DIR}/tests/valgrind_decompose_target.cpp")
add_executable(${BINARY} ${TEST_SOURCES} test_bbox.cpp)
target_link_libraries(${BINARY} PRIVATE gtest gtest_main CGAL::CGAL CGAL::Eigen3_support spdlog::spdlog)
target_include_directories(${BINARY} PUBLIC ../code)
target_link_libraries(${BINARY} PUBLIC ${CMAKE_PROJECT_NAME}_lib )
# copy testfiles
file(GLOB TEST_FILES ${CMAKE_CURRENT_SOURCE_DIR}/files/*.txt ${CMAKE_CURRENT_SOURCE_DIR}/files/*.off ${CMAKE_CURRENT_SOURCE_DIR}/files/*.ol)
foreach(test_source ${TEST_FILES})
    get_filename_component(test_file_name "${test_source}" NAME)
    configure_file(${test_source} ${CMAKE_CURRENT_BINARY_DIR}/files/${test_file_name} COPYONLY)
endforeach()
