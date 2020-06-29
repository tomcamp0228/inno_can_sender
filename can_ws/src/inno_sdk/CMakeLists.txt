cmake_minimum_required(VERSION 3.5)
project(inno_sdk)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

install(
  DIRECTORY include
#   EXPORT export_${PROJECT_NAME}
  DESTINATION include
)
install(
  FILES lib_emuc2_64.a
  DESTINATION lib 
)

ament_export_include_directories(include)
ament_package()