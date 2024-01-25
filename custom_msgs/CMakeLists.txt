cmake_minimum_required(VERSION 3.8)
project(custom_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(srv_files
"srv/ChangeGripper.srv"
"srv/PickupPart.srv"
"srv/PickupTray.srv"
"srv/PlacingTray.srv"
"srv/PlacingPart.srv"
"srv/RemovePart.srv"
)

set(msg_files
"msg/PartDelivery.msg"
"msg/TrayDelivery.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${srv_files}
    ${msg_files}
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
