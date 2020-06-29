cmake_minimum_required(VERSION 3.5)
project(inno_can_send)

## Set Boost paths
set(Boost_INCLUDE_DIRS /usr/include/boost)
set(Boost_LIBRARIES_DIRS /usr/lib/x86_64-linux-gnu)
## Set IBEO paths
set (INNO_DIR1      ${CMAKE_CURRENT_SOURCE_DIR}/../../install/inno_sdk/include)
set (INNO_LIB1      ${CMAKE_CURRENT_SOURCE_DIR}/../../install/inno_sdk/lib/lib_emuc2_64.a)
set (INNO_LIB_DIRS  ${CMAKE_CURRENT_SOURCE_DIR}/../../install/inno_sdk/lib)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

SET(CMAKE_CXX_FLAGS "-O2 -g ${CMAKE_CXX_FLAGS}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


## Find macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(inno_sdk REQUIRED)


find_package(Boost REQUIRED COMPONENTS system chrono regex thread program_options date_time)

find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(luuid REQUIRED)

###########
## Build ##
###########
link_directories(
  ${INNO_LIB_DIRS}
)

add_executable(inno_can_send src/inno_can_send.cpp)
target_include_directories(inno_can_send PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${INNO_DIR1}
)
target_link_libraries(inno_can_send 
  ${Boost_LIBRARIES}
  ${INNO_LIB1}
  # ${inno_sdk_LIBRARIES}
  ${std_interfaces}
)

ament_target_dependencies(
  inno_can_send
  "std_msgs"
  "rclcpp"
  "inno_sdk"
  "nav_msgs"

)


install(TARGETS inno_can_send
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
# install(DIRECTORY record
#   DESTINATION share/${PROJECT_NAME}
# )
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()