#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ariac_moveit_config.parameters import generate_parameters

def generate_launch_description():
    ld = LaunchDescription()

    submit_orders_agv1 = Node(
        package="rwa67",
        executable="submit_orders_exe",
        parameters=[{'agv_number':1}],   # parameter
        name='submit_orders_agv1'            # node remapping
    )

    submit_orders_agv2 = Node(
        package="rwa67",
        executable="submit_orders_exe",
        parameters=[{'agv_number':2}],   # parameter
        name='submit_orders_agv2',            # node remapping
        remappings=[    # topic remapping
            ('/ariac/agv1_status', '/ariac/agv2_status')
        ]
    )

    submit_orders_agv3 = Node(
        package="rwa67",
        executable="submit_orders_exe",
        parameters=[{'agv_number':3}],   # parameter
        name='submit_orders_agv3',            # node remapping
        remappings=[    # topic remapping
            ('/ariac/agv1_status', '/ariac/agv3_status')
        ]
    )

    submit_orders_agv4 = Node(
        package="rwa67",
        executable="submit_orders_exe",
        parameters=[{'agv_number':4}],   # parameter
        name='submit_orders_agv4',            # node remapping
        remappings=[    # topic remapping
            ('/ariac/agv1_status', '/ariac/agv4_status')
        ]
    )

    # start competition
    start_comp = Node(
        package="rwa67",
        executable="start_comp.py",
    )

    # ship order node
    ship_order = Node(
        package="rwa67",
        executable="ship_order",
        parameters=[{"total_agv_number":4}]
    )

    # change gripper server
    change_gripper_server = Node(
        package="rwa67",
        executable="change_gripper_server",
        parameters=[{"emulate_only":False}]
    )
    # pickup tray server
    pickup_tray_server = Node(
        package="rwa67",
        executable="start_pickup_tray_server.py",
    )
    # pickup part server
    pickup_part_server = Node(
        package="rwa67",
        executable="start_pickup_part_server.py",
    )
    # floor robot server
    floor_robot_server = Node(
        package='rwa67',
        executable='floor_robot_server',
        # output="screen",
        parameters=generate_parameters()
    )
    # moveit node
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_moveit_config"), "/launch", "/ariac_robots_moveit.launch.py"]
        )
    )
    # Actions
    ld.add_action(start_comp)
    ld.add_action(ship_order)
    ld.add_action(submit_orders_agv1)
    ld.add_action(submit_orders_agv2)
    ld.add_action(submit_orders_agv3)
    ld.add_action(submit_orders_agv4)
    ld.add_action(change_gripper_server)
    # ld.add_action(moveit)
    ld.add_action(floor_robot_server)
    
    return ld

 