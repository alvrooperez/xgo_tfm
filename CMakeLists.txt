cmake_minimum_required(VERSION 3.16)
project(xgo_driver_cpp)

# Requerir C++20 estricto
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Opciones del compilador
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencias de ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# -----------------------------------------------------------------------------
# 1. Librería Core del Driver (C++ puro desacoplado)
# -----------------------------------------------------------------------------
add_library(xgo_driver
  src/xgo_mini2s.cpp
  src/serial.cpp
)

target_include_directories(xgo_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# -----------------------------------------------------------------------------
# 2. Nodo de ROS 2 (se compilará cuando creemos src/xgo_driver_node.cpp)
# -----------------------------------------------------------------------------
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/xgo_driver_node.cpp")
  add_executable(xgo_driver_node src/xgo_driver_node.cpp)
  target_link_libraries(xgo_driver_node
    xgo_driver
    ${rclcpp_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${nav_msgs_LIBRARIES}
  )
  ament_target_dependencies(xgo_driver_node
    rclcpp
    geometry_msgs
    sensor_msgs
    nav_msgs
  )
  install(TARGETS xgo_driver_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()


# -----------------------------------------------------------------------------
# 4. Instalación y Exportación para ROS 2
# -----------------------------------------------------------------------------
install(TARGETS xgo_driver
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(xgo_driver)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
