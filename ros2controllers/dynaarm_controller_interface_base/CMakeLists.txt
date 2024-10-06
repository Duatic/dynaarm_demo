cmake_minimum_required(VERSION 3.5)
project(dynaarm_controller_interface_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(PACKAGE_DEPENDS
  hardware_interface
  rclcpp_lifecycle
  controller_interface
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${PACKAGE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  ${PROJECT_NAME}
  SHARED
  src/dynaarm_controller_interface_base.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(${PROJECT_NAME} ${PACKAGE_DEPENDS})
target_compile_definitions(${PROJECT_NAME} PRIVATE "dynaarm_controller_interface_base_BUILDING_DLL")

install(DIRECTORY include/
  DESTINATION include
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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