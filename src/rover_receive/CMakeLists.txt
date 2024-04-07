cmake_minimum_required(VERSION 3.5)  
project(rover_receive)

# Default to C++14  
if(NOT CMAKE_CXX_STANDARD)  
    set(CMAKE_CXX_STANDARD 14)  
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")  
    add_compile_options(-Wall -Wextra -Wpedantic)  
endif()

# Find packages  
find_package(ament_cmake REQUIRED)  
find_package(rclcpp REQUIRED)  
find_package(std_msgs REQUIRED)

# Include libserial-dev directory
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL libserial)
include_directories(${LIBSERIAL_INCLUDE_DIRS})

# Executable  
add_executable(receive_node_nano src/receive_node_nano.cpp)

# Dependencies  
ament_target_dependencies(receive_node_nano rclcpp std_msgs)

# Link with WiringPi and wiringSerial  
target_link_libraries(receive_node_nano wiringPi wiringSerial ${LIBSERIAL_LIBRARIES})

# Installation  
install(TARGETS  
receive_node_nano  
DESTINATION lib/${PROJECT_NAME})

# Package export  
ament_package()
