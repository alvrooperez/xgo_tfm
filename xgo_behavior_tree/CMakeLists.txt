cmake_minimum_required(VERSION 3.16)
project(xgo_behavior_tree)

# Estándar C++20 estricto
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------------------
# 1. Dependencias
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)

# -----------------------------------------------------------------------------
# 2. Rutas de cabeceras
# -----------------------------------------------------------------------------
include_directories(
  include
)

# -----------------------------------------------------------------------------
# 3. Ejecutable principal del Behavior Tree
# -----------------------------------------------------------------------------
add_executable(bt_executor_node src/bt_executor_node.cpp)

target_include_directories(bt_executor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(bt_executor_node
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  nav2_msgs
  behaviortree_cpp
  ament_index_cpp
)

install(TARGETS bt_executor_node
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
# 4. Instalación de ficheros de configuración (Árboles XML)
# -----------------------------------------------------------------------------
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY include/
  DESTINATION include
)

# -----------------------------------------------------------------------------
# 4. Exportación de dependencias del paquete
# -----------------------------------------------------------------------------
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  nav2_msgs
  behaviortree_cpp
)

ament_package()
