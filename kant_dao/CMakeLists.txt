cmake_minimum_required(VERSION 3.5)
project(kant_dao)

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
find_package(kant_knowledge_base REQUIRED)
find_package(std_srvs REQUIRED)
find_package(simple_node REQUIRED)

# C++
include_directories(include)
include_directories(src)

set(LIB ${CMAKE_PROJECT_NAME}_lib)
set(SOURCES
  src/kant_dao/ros2_dao/ros2_pddl_type_dao.cpp
  src/kant_dao/ros2_dao/ros2_pddl_object_dao.cpp
  src/kant_dao/ros2_dao/ros2_pddl_predicate_dao.cpp
  src/kant_dao/ros2_dao/ros2_pddl_proposition_dao.cpp
  src/kant_dao/ros2_dao/ros2_pddl_action_dao.cpp
  src/kant_dao/mongo_dao/mongo_pddl_type_dao.cpp
  src/kant_dao/mongo_dao/mongo_pddl_object_dao.cpp
  src/kant_dao/mongo_dao/mongo_pddl_predicate_dao.cpp
  src/kant_dao/mongo_dao/mongo_pddl_proposition_dao.cpp
  src/kant_dao/mongo_dao/mongo_pddl_action_dao.cpp
  src/kant_dao/dao_factory/dao_factories/ros2_dao_factory.cpp
  src/kant_dao/dao_factory/dao_factories/mongo_dao_factory.cpp
  src/kant_dao/dao_factory/dao_factory_method.cpp
  src/kant_dao/parameter_loader.cpp
)

set(DEPENDENCIES
    rclcpp 
    kant_interfaces 
    kant_dto
    kant_knowledge_base
    std_srvs
    simple_node
)

add_library(${LIB} STATIC ${SOURCES})

install(TARGETS ${LIB}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_target_dependencies(${LIB} ${DEPENDENCIES})

target_include_directories(${LIB} PUBLIC "/usr/local/include/mongocxx/v_noabi")
target_link_libraries(${LIB} mongocxx)

target_include_directories(${LIB} PUBLIC "/usr/local/include/bsoncxx/v_noabi")
target_link_libraries(${LIB} bsoncxx)


ament_export_include_directories(include)
ament_export_libraries(${LIB})
ament_export_dependencies(${DEPENDENCIES})


add_executable(example_node
  src/example_node.cpp
)
ament_target_dependencies(example_node ${DEPENDENCIES})
target_link_libraries(example_node ${LIB})
install(TARGETS 
  example_node
  DESTINATION lib/${PROJECT_NAME}
)

# Python
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  kant_dao/example_node.py
  DESTINATION lib/${PROJECT_NAME}
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