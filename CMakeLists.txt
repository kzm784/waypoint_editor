cmake_minimum_required(VERSION 3.8)
project(waypoint_editor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(pluginlib REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(panel_headers
  include/waypoint_editor/waypoint_editor_panel.hpp
)
qt5_wrap_cpp(waypoint_editor_moc ${panel_headers})

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

add_library(${PROJECT_NAME} SHARED
  src/waypoint_editor_tool.cpp
  src/waypoint_editor_panel.cpp
  ${waypoint_editor_moc}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  rviz_default_plugins
  pluginlib
  interactive_markers
  visualization_msgs
  geometry_msgs
  std_srvs
  std_msgs
  nav2_msgs
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY icons
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch rviz data
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rviz_common
  rviz_default_plugins
  pluginlib
  interactive_markers
  visualization_msgs
  geometry_msgs
  std_srvs
  std_msgs
  nav2_msgs
)

ament_package()
