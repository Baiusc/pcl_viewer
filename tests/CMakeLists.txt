add_executable(cv_test_core test_core.cpp)
target_link_libraries(cv_test_core PRIVATE cv_core cv_render cv_io)
add_test(NAME core_frame_mapping COMMAND cv_test_core)

add_executable(cv_test_io test_io.cpp)
target_link_libraries(cv_test_io PRIVATE cv_core cv_io)
target_compile_definitions(cv_test_io PRIVATE
    CV_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../data")
add_test(NAME io_probe_and_index COMMAND cv_test_io)

add_executable(cv_test_color test_color.cpp)
target_link_libraries(cv_test_color PRIVATE cv_render cv_core)
add_test(NAME color_mapping COMMAND cv_test_color)
