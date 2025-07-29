cmake_minimum_required(VERSION 3.5)
set (PROJECT_NAME param_estimation)
project(${PROJECT_NAME})

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Default build type to Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ROS and external dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()


# Include directories
include_directories(
  include
  include/${PROJECT_NAME}            
)

# Source files
set(SOURCE_CPP_FILES
  src/param_estimation.cpp
)

# Create library
add_library(${PROJECT_NAME} SHARED ${SOURCE_CPP_FILES})

# Link dependencies properly
ament_target_dependencies(${PROJECT_NAME} rclcpp)

# Export library headers
target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Executable target
add_executable(main src/main.cpp)
ament_target_dependencies(main rclcpp)
target_link_libraries(main ${PROJECT_NAME})

# Export
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

# Install
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
