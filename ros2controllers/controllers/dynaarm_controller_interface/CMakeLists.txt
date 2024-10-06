cmake_minimum_required(VERSION 3.5)
project(dynaarm_controller_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(PACKAGE_DEPENDS
  dynaarm_controller_interface_base
)

include(cmake/pinocchio.cmake)
message(STATUS "Pinocchio flags: " ${PINOCCHIO_FLAGS})

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(pinocchio REQUIRED NO_MODULE)
foreach(Dependency IN ITEMS ${PACKAGE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

install(DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(
  ${PACKAGE_DEPENDS}
  Eigen3
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_package()