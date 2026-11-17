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

add_executable(cv_test_beta_stream test_beta_stream.cpp)
target_link_libraries(cv_test_beta_stream PRIVATE cv_stream cv_core)
add_test(NAME beta_packet_parse COMMAND cv_test_beta_stream)

add_executable(cv_test_beta_calibration test_beta_calibration.cpp)
target_link_libraries(cv_test_beta_calibration PRIVATE cv_stream cv_core)
target_compile_definitions(cv_test_beta_calibration PRIVATE
    CV_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../data")
add_test(NAME beta_calibration_modes COMMAND cv_test_beta_calibration)

add_executable(cv_test_discovery test_device_discovery.cpp)
target_link_libraries(cv_test_discovery PRIVATE cv_stream cv_core)
add_test(NAME device_broadcast_parse COMMAND cv_test_discovery)

add_executable(cv_test_ui_overlay test_ui_overlay.cpp)
target_link_libraries(cv_test_ui_overlay PRIVATE cv_render cv_core viewer_deps)
add_test(NAME ui_button_hit_log COMMAND cv_test_ui_overlay)

add_executable(cv_test_online test_online.cpp)
target_link_libraries(cv_test_online PRIVATE cv_stream cv_preprocess)
target_include_directories(cv_test_online PRIVATE ${CV_LZ4_INCLUDE})
add_test(NAME online_range_and_calibration COMMAND cv_test_online)
set_tests_properties(online_range_and_calibration PROPERTIES TIMEOUT 30)

add_executable(cv_test_anchor_table test_anchor_table.cpp)
target_link_libraries(cv_test_anchor_table PRIVATE cv_stream cv_core)
target_compile_definitions(cv_test_anchor_table PRIVATE
    CV_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../data")
add_test(NAME beta_anchor_table_layout COMMAND cv_test_anchor_table)
