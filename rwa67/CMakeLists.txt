cmake_minimum_required(VERSION 3.8)
project(rwa67)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(ariac_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(robot_commander_msgs REQUIRED)

install(PROGRAMS
    nodes/start_comp.py
    nodes/start_pickup_part_server.py
    nodes/start_pickup_tray_server.py
    DESTINATION lib/${PROJECT_NAME})
    
# ship_order node
add_executable(ship_order src/ship_order.cpp)
ament_target_dependencies(ship_order rclcpp ariac_msgs std_msgs std_srvs)
target_include_directories(ship_order PUBLIC include)
install(TARGETS ship_order DESTINATION lib/${PROJECT_NAME})

# submit_order node
add_executable(submit_orders_exe src/submit_orders.cpp)
ament_target_dependencies(submit_orders_exe rclcpp ariac_msgs)
target_include_directories(submit_orders_exe PUBLIC include)
install(TARGETS submit_orders_exe DESTINATION lib/${PROJECT_NAME})

# change gripper node
add_executable(change_gripper_server src/change_gripper_server.cpp)
ament_target_dependencies(change_gripper_server rclcpp ariac_msgs custom_msgs robot_commander_msgs)
target_include_directories(change_gripper_server PUBLIC include)
install(TARGETS change_gripper_server DESTINATION lib/${PROJECT_NAME})

# floor robot service
# create a variable for all dependencies
set(FLOOR_ROBOT_INCLUDE_DEPENDS
  rclcpp 
  std_msgs
  ariac_msgs
  tf2_ros
  geometry_msgs
  tf2
  orocos_kdl
  tf2_ros
  tf2_geometry_msgs
  moveit_msgs
  geometric_shapes
  moveit_ros_planning_interface
  robot_commander_msgs
  custom_msgs
)

# find all dependencies
foreach(dependency IN ITEMS ${FLOOR_ROBOT_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

add_executable(floor_robot_server src/floor_robot_main.cpp src/floor_robot.cpp)
ament_target_dependencies(floor_robot_server  ${FLOOR_ROBOT_INCLUDE_DEPENDS})
target_include_directories(floor_robot_server PUBLIC include)
install(TARGETS floor_robot_server DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# directories install
install(DIRECTORY
  include
  launch
  config
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
