cmake_minimum_required(VERSION 3.5)
project(mpu9250driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(mpu9250driver src/mpu9250driver.cpp src/mpu9250sensor.cpp)
ament_target_dependencies(mpu9250driver rclcpp sensor_msgs)
target_link_libraries(mpu9250driver i2c)
target_include_directories(
  mpu9250driver PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                       $<INSTALL_INTERFACE:include>)

install(TARGETS mpu9250driver DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME}/)
ament_package()
