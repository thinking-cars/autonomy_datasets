cmake_minimum_required(VERSION 3.12.0 FATAL_ERROR)
project(autonomy_datasets_rviz_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

find_package(ament_cmake REQUIRED)
find_package(autonomy_datasets_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# The moc output of every panel header is included by its own source file (see the bottom of
# src/playback_panel.cpp), so that Qt generates no translation unit of its own. Generated Qt code
# does not satisfy the clang-tidy rules of this repository, and a translation unit that only the
# code generator ever sees would have to be excluded from linting again.
set(CMAKE_AUTOMOC ON)

add_library(${PROJECT_NAME} SHARED
    src/playback_panel.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${Qt5Widgets_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    Qt5::Widgets
)

ament_target_dependencies(${PROJECT_NAME}
    PUBLIC
    autonomy_datasets_msgs
    pluginlib
    rclcpp
    rviz_common
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    autonomy_datasets_msgs
    pluginlib
    rclcpp
    rviz_common
)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Only report clang-tidy findings in the headers of this package. The default filter is
  # "include/<package>/.*", which also matches what Qt generates below
  # "<package>_autogen/include/<package>/".
  set(ament_cmake_clang_tidy_HEADER_FILTER "${CMAKE_CURRENT_SOURCE_DIR}/include/.*")

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_clang_format_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../.vscode/format/.clang-format)
  set(ament_cmake_clang_tidy_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../.vscode/lint/.clang-tidy)
  set(ament_cmake_flake8_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../.vscode/lint/ament_flake8.ini)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
