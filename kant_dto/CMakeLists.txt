cmake_minimum_required(VERSION 3.5)
project(kant_dto)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# C++
include_directories(include)
include_directories(src)

set(LIB ${CMAKE_PROJECT_NAME}_lib)
set(SOURCES
  src/kant_dto/pddl_action_dto.cpp
  src/kant_dto/pddl_condition_effect_dto.cpp
  src/kant_dto/pddl_object_dto.cpp
  src/kant_dto/pddl_predicate_dto.cpp
  src/kant_dto/pddl_proposition_dto.cpp
  src/kant_dto/pddl_type_dto.cpp
)

add_library(${LIB} STATIC ${SOURCES})

install(TARGETS ${LIB}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${LIB})

# Python
ament_python_install_package(${PROJECT_NAME})

# Tests
install(
  DIRECTORY tests/python/
  DESTINATION pytests/
)

install(
  PROGRAMS tests/python/run_pytests.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()