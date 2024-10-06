cmake_minimum_required(VERSION 3.8)
project(dynaarm_position_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set(PACKAGE_DEPENDS
  generate_parameter_library
  rclcpp
  pluginlib
  realtime_tools
  dynaarm_controller_interface
  sensor_msgs
  geometry_msgs
  dynaarm_msgs
)

include(cmake/pinocchio.cmake)
message(STATUS "Pinocchio flags: " ${PINOCCHIO_FLAGS})
find_package(pinocchio REQUIRED NO_MODULE)

foreach(Dependency IN ITEMS ${PACKAGE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(${PROJECT_NAME}_parameters
  src/${PROJECT_NAME}.yaml
)

add_library(${PROJECT_NAME} SHARED   
  src/dynaarm_position_controller.cpp)
target_include_directories(${PROJECT_NAME}
    PUBLIC
      include
    PRIVATE
      ${pinocchio_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC ${PROJECT_NAME}_parameters)
target_link_libraries(${PROJECT_NAME} PRIVATE ${pinocchio_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${PACKAGE_DEPENDS})
pluginlib_export_plugin_description_file(controller_interface controller_interface_plugin.xml)
target_compile_options(${PROJECT_NAME} PRIVATE ${PINOCCHIO_FLAGS})

install(
  DIRECTORY include/
  DESTINATION include/
)
install(TARGETS ${PROJECT_NAME}  
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  ${PACKAGE_DEPENDS}
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_package()