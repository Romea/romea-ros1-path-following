find_package(ament_cmake_gtest REQUIRED)

get_filename_component(TEST_DIR_SRC ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)
configure_file(test_helper.h.in test_helper.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

ament_add_gtest(${PROJECT_NAME}_test_lateral_control test_lateral_control.cpp)
target_link_libraries(${PROJECT_NAME}_test_lateral_control ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_sliding_observer test_sliding_observer.cpp)
target_link_libraries(${PROJECT_NAME}_test_sliding_observer ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_path_following_factory test_path_following_factory.cpp)
target_link_libraries(${PROJECT_NAME}_test_path_following_factory ${PROJECT_NAME})
