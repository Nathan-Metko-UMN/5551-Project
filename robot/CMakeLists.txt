cmake_minimum_required(VERSION 3.16)
project(robot LANGUAGES CXX)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# find dependencies
set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  controller_interface
  rclcpp
  rclcpp_lifecycle
)

# Jazzy ships ros2_control 4.x
find_package(controller_manager 4.0.0 REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(Eigen3 REQUIRED)
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(robot_controller SHARED
  controller/robot_controller.cpp
)
add_executable(wire_generator src/wire_generator.cpp)

target_include_directories(robot_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_include_directories(wire_generator  PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(robot_controller PUBLIC
  controller_interface::controller_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  Eigen3::Eigen
)
target_link_libraries(wire_generator   PRIVATE rclcpp::rclcpp Eigen3::Eigen)

# Register the controller with pluginlib
pluginlib_export_plugin_description_file(controller_interface robot_controller.xml)

## INSTALL
install(
  DIRECTORY controller/include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  DIRECTORY launch config discription
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS robot_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS wire_generator RUNTIME DESTINATION lib/${PROJECT_NAME})


## EXPORTS
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${CONTROLLER_INCLUDE_DEPENDS})
ament_package()
