cmake_minimum_required(VERSION 3.5)
project(multi_nav2)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(action_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)


find_package(ZMQ)
if(ZMQ_FOUND)
    message(STATUS "ZeroMQ found.")
    add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  action_msgs
  visualization_msgs
  nav_msgs
  geometry_msgs
  nav2_costmap_2d
  rclcpp_action
  nav2_msgs
  ament_index_cpp
)

include_directories(include ${ZMQ_INCLUDE_DIRS})

add_library(explored_bt_node SHARED src/Explored.cpp)
add_library(move_bt_node SHARED src/Move.cpp)
add_library(selectpf_bt_node SHARED src/SelectPF.cpp)
list(APPEND plugin_libs
  explored_bt_node
  move_bt_node
  selectpf_bt_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()


add_executable(check_pf src/check_pf.cpp)
ament_target_dependencies(check_pf ${dependencies})
target_link_libraries(check_pf ${ZMQ_LIBRARIES})

add_executable(patrolling_main src/patrolling_main.cpp)
ament_target_dependencies(patrolling_main ${dependencies})
target_link_libraries(patrolling_main ${ZMQ_LIBRARIES})

install(TARGETS
  ${plugin_libs}
  check_pf
  patrolling_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  # RUNTIME DESTINATION bin
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY multi_nav_tree
  DESTINATION share/${PROJECT_NAME}
)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  add_subdirectory(tests)
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
