cmake_minimum_required(VERSION 3.5)
project(kant_knowledge_base)

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
find_package(kant_dto REQUIRED)
find_package(kant_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(simple_node REQUIRED)

# C++
include_directories(include)
include_directories(src)

set(LIB ${CMAKE_PROJECT_NAME}_lib)
set(SOURCES
  src/kant_knowledge_base/knowledge_base/knowledge_base.cpp
  src/kant_knowledge_base/parser/dto_msg_parser.cpp
  src/kant_knowledge_base/parser/msg_dto_parser.cpp
  src/kant_knowledge_base/knowledge_base/knowledge_base_node.cpp
)

set(DEPENDENCIES
    rclcpp 
    kant_interfaces 
    kant_dto
    std_srvs
    simple_node
)

add_library(${LIB} STATIC ${SOURCES})

ament_target_dependencies(${LIB} ${DEPENDENCIES})


add_executable(knowledge_base_node
  src/knowledge_base_node_main.cpp
)
ament_target_dependencies(knowledge_base_node ${DEPENDENCIES})
target_link_libraries(knowledge_base_node ${LIB})

install(TARGETS ${LIB} 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS 
  knowledge_base_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${LIB})
ament_export_dependencies(${DEPENDENCIES})


# Python
ament_python_install_package(${PROJECT_NAME})


install(PROGRAMS
  kant_knowledge_base/knowledge_base_node_main.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME knowledge_base_node.py
)

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