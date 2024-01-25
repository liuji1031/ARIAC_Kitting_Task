import time
from typing import List

import copy
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.task import Future
from std_msgs.msg import UInt8, String
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped, Pose, Quaternion

from ariac_msgs.msg import (
    AdvancedLogicalCameraImage as AdvancedLogicalCameraImageMsg,
    CompetitionState as CompetitionStateMsg,
    Part as PartMsg,
    PartPose as PartPoseMsg,
    Order as OrderMsg,
    AssemblyPart as AssemblyPartMsg,
    AGVStatus as AGVStatusMsg,
    AssemblyTask as AssemblyTaskMsg,
    KittingTask,
    KittingPart,
    KitTrayPose,
    QualityIssue
)

from ariac_msgs.srv import (
    ChangeGripper as AriacChangeGripper ,
    VacuumGripperControl,
    PerformQualityCheck
)

from custom_msgs.srv import (
    ChangeGripper as CustomChangeGripper,
    PickupPart,
    PickupTray,
    PlacingPart,
    PlacingTray,
    RemovePart
)

from std_srvs.srv import Trigger

from robot_commander_msgs.srv import(
    MoveRobotToTray,
    MoveTrayToAGV,
    MoveRobotToTable
)

from rwa67.action_tree import ActionNode, ActionTree

from rwa67.utils import (
    multiply_pose,
    rpy_from_quaternion,
    rad_to_deg_str,
    AdvancedLogicalCameraImage,
    dist_two_poses,
    print_assigned_order,
    print_poses,
    Tray,
    Part,
    min_ind_pose_dist,
    min_sum_pose_dist,
    gen_pose
)

from typing import List

class CompetitionInterface(Node):
    '''
    Class for a competition interface node.

    Args:
        Node (rclpy.node.Node): Parent class for ROS nodes

    Raises:
        KeyboardInterrupt: Exception raised when the user uses Ctrl+C to kill a process
    '''

    _competition_states = {
        CompetitionStateMsg.IDLE: 'idle',
        CompetitionStateMsg.READY: 'ready',
        CompetitionStateMsg.STARTED: 'started',
        CompetitionStateMsg.ORDER_ANNOUNCEMENTS_DONE: 'order_announcements_done',
        CompetitionStateMsg.ENDED: 'ended',
    }
    '''Dictionary for converting CompetitionState constants to strings'''

    _part_colors = {
        PartMsg.RED: 'red',
        PartMsg.BLUE: 'blue',
        PartMsg.GREEN: 'green',
        PartMsg.ORANGE: 'orange',
        PartMsg.PURPLE: 'purple',
    }

    _part_types = {
        PartMsg.BATTERY: 'battery',
        PartMsg.PUMP: 'pump',
        PartMsg.REGULATOR: 'regulator',
        PartMsg.SENSOR: 'sensor',
    }

    _destinations = {
        AGVStatusMsg.KITTING: 'kitting station',
        AGVStatusMsg.ASSEMBLY_FRONT: 'front assembly station',
        AGVStatusMsg.ASSEMBLY_BACK: 'back assembly station',
        AGVStatusMsg.WAREHOUSE: 'warehouse',
    }

    _stations = {
        AssemblyTaskMsg.AS1: 'assembly station 1',
        AssemblyTaskMsg.AS2: 'assembly station 2',
        AssemblyTaskMsg.AS3: 'assembly station 3',
        AssemblyTaskMsg.AS4: 'assembly station 4',
    }

    # status of each order
    _ORDER_IN_PROCESS = 0
    _ORDER_COMPLETED = 1
    _ORDER_SHIPPED = 2
    _ORDER_SUBMITTED = 3

    def __init__(self):
        super().__init__('competition_interface')

        sim_time = Parameter(
            "use_sim_time",
            rclpy.Parameter.Type.BOOL,
            True
        )

        self.set_parameters([sim_time])
        # Service client for starting the competition
        self._start_competition_client = self.create_client(
            Trigger, '/ariac/start_competition')
        
        # Service client for ending the competition
        self._end_competition_client = self.create_client(
            Trigger, '/ariac/end_competition')

        # client to activate/deactivate the vacuum gripper
        self._set_gripper_state_cli = self.create_client(
            VacuumGripperControl,
            '/ariac/floor_robot_enable_gripper')
        
        # Subscriber to the competition state topic
        self._competition_state_sub = self.create_subscription(
            CompetitionStateMsg,
            '/ariac/competition_state',
            self._competition_state_cb,
            10)
        
        # Store the state of the competition
        self._competition_state: CompetitionStateMsg = None

        cam_callbackgrp = MutuallyExclusiveCallbackGroup()
         # Subscriber to the logical camera topic (right_bins_camera)
        self.right_bins_camera_sub = self.create_subscription(
            AdvancedLogicalCameraImageMsg,
            '/ariac/sensors/right_bins_camera/image',
            self._right_bins_camera_cb,
            qos_profile_sensor_data,
            callback_group=cam_callbackgrp)
        
        # Subscriber to the logical camera topic (left_bins_camera)
        self.left_bins_camera_sub = self.create_subscription(
            AdvancedLogicalCameraImageMsg,
            '/ariac/sensors/left_bins_camera/image',
            self._left_bins_camera_cb,
            qos_profile_sensor_data,
            callback_group=cam_callbackgrp)
        
        # Subscriber to the logical camera topic kts1_camera)
        self.kts1_camera_sub = self.create_subscription(
            AdvancedLogicalCameraImageMsg,
            '/ariac/sensors/kts1_camera/image',
            self._kts1_camera_cb,
            qos_profile_sensor_data,
            callback_group=cam_callbackgrp)
        
        # Subscriber to the logical camera topic kts2_camera)
        self.kts2_camera_sub = self.create_subscription(
            AdvancedLogicalCameraImageMsg,
            '/ariac/sensors/kts2_camera/image',
            self._kts2_camera_cb,
            qos_profile_sensor_data,
            callback_group=cam_callbackgrp)
        
        self._cam_updated = dict()
        
        # Store each camera image as an AdvancedLogicalCameraImage object
        self._camera_image: AdvancedLogicalCameraImage = None

        # create a subscriber to recieve the order message
        self.orders_sub = self.create_subscription(OrderMsg, '/ariac/orders', self._orders_cb, 10)

        # Flag for parsing incoming orders
        self._parse_incoming_order = False

        # List of orders
        self._orders = []

        # create publisher for anouncing which agv should go to the warehouse
        self._move_agv_pub = self.create_publisher(UInt8, 'move_agv_warehouse', 10)

        # publish submitted orders
        self.submitted_order_sub = self.create_subscription(String,'/ariac/submitted_order',self.submitted_order_sub_cb, 10)

        # list to store all discovered tray and parts
        self._parts = []
        self._part_count = 0
        self._trays = []
        self._check_for_new_parts = True
        self._activated_gripper = False
        self._deactivated_gripper = False
        self._moved_robot_to_tray = False
        self._activating_gripper = False
        self._deactivating_gripper = False


        # create listners for AGV poses
        # Create a transform buffer and listener
        self._tf_buffer_agv = [Buffer() for _ in range(4)]
        self._tf_listener_agv = [TransformListener(tf_buffer, self) for tf_buffer in self._tf_buffer_agv]

        # create listners for floor robot base
        self._tf_buffer_floor_robot = Buffer()
        self._tf_listener_floor_robot = TransformListener(self._tf_buffer_floor_robot, self)

        # create listner for kit station pose
        self._tf_buffer_kts = [Buffer() for _ in range(2)]
        self._tf_listener_kts = [TransformListener(tf_buffer, self) for tf_buffer in self._tf_buffer_kts]

        # Listen to the transform between frames periodically
        self._listener_timer = self.create_timer(1, self._agv_pose_listner_cb)
        self._listener_timer = self.create_timer(0.02, self._listener_floor_robot_cb)

        # store the floor robot base position
        self._floor_base = Pose()
        self._floor_base_updated = False
        
        # store the kitting station position
        self._kts_poses = []
        self._kts_poses.append(gen_pose(x=-1.3,y=-5.84,z=0.0,roll=0.0,pitch=0.0,yaw=3.14)) # kitting table 1
        self._kts_poses.append(gen_pose(x=-1.3,y=+5.84,z=0.0,roll=0.0,pitch=0.0,yaw=0.0)) # kitting table 2

        # list to store agv poses
        self._agv_poses = [Pose() for _ in range(4)]
        self._agv_pose_updated = False

        # dict to store all action trees created for each order
        self._action_trees = dict()


        # timer for creating action trees
        self._create_tree_timer = self.create_timer(0.1, self._create_tree_cb)

        # the queue for executing actions
        self._action_queue = []

        # timer for managing action queue
        self._queue_manager_timer = self.create_timer(0.1, self._manage_action_queue_cb)

        # change gripper client
        chg_gripper_cb_group = MutuallyExclusiveCallbackGroup()
        self._change_gripper_client = self.create_client(CustomChangeGripper,
                                                         "/ariac/custom_change_gripper",
                                                         callback_group=chg_gripper_cb_group)
        
        # client for pick up service
        pickup_cb_group = MutuallyExclusiveCallbackGroup()
        self._pickup_tray_client = self.create_client(MoveRobotToTray,
                                                         "/commander/move_robot_to_tray",
                                                         callback_group=pickup_cb_group)
        self._pickup_part_client = self.create_client(PickupPart,
                                                         "/commander/pickup_part",
                                                         callback_group=pickup_cb_group)
        
        # client for place service
        place_cb_group = MutuallyExclusiveCallbackGroup()
        self._place_tray_client = self.create_client(PlacingTray,
                                                         "/commander/place_tray_on_agv",
                                                         callback_group=place_cb_group)
        self._place_part_client = self.create_client(PlacingPart,
                                                         "/commander/place_part_on_tray",
                                                         callback_group=place_cb_group)
        
        # client for removing faulty part from AGV
        remove_part_cbg = MutuallyExclusiveCallbackGroup()
        self._remove_part_client = self.create_client(RemovePart,
                                                      "/commander/remove_part_from_agv",
                                                      callback_group=remove_part_cbg)
        
        # client for moving robot to table
        move_robot_table_cbg = MutuallyExclusiveCallbackGroup()
        self._move_robot_to_table_cli = self.create_client(MoveRobotToTable,
                                                           "/commander/move_robot_to_table",
                                                           callback_group=move_robot_table_cbg)

        # client for moving robot to home
        move_home_cb_group = MutuallyExclusiveCallbackGroup()
        self._move_robot_to_home_cli = self.create_client(MoveRobotToTable,
                                                           "/commander/move_robot_home",
                                                           callback_group=move_home_cb_group)

        # coefficients for weighting different costs
        self._k_cost_dist = 1.0 # coefficient for distance cost
        self._k_cost_chg_gripper = 100.0 # coefficient for change gripper cost

        # track current the gripper on the robot
        self._curr_gripper = AriacChangeGripper.Request.PART_GRIPPER # competition always starts with tray gripper

        # dict monitoring the order status
        self._order_status = dict()

        # timer for tracking order status
        self._order_status_timer = self.create_timer(0.1, self._check_order_status_cb)

        # client for quality check
        quality_check_cbg = MutuallyExclusiveCallbackGroup()
        self._quality_check_cli = self.create_client(PerformQualityCheck,
                                                     "/ariac/perform_quality_check",
                                                     callback_group=quality_check_cbg)
        self._pass_quality_check = dict() # key is kitting order, value is true or false

    @property
    def camera_image(self):
        return self._camera_image
    
    @property
    def orders(self):
        return self._orders
        
    @property
    def parse_incoming_order(self):
        return self._parse_incoming_order

    @parse_incoming_order.setter
    def parse_incoming_order(self, value):
        self._parse_incoming_order = value

    @property
    def competition_state(self):
        return self._competition_state
    
    def start_competition(self):
        '''Function to start the competition.
        '''
        self.get_logger().info('Waiting for competition to be ready')

        if self._competition_state == CompetitionStateMsg.STARTED:
            return
        # Wait for competition to be ready
        while self._competition_state != CompetitionStateMsg.READY:
            try:
                rclpy.spin_once(self)
            except KeyboardInterrupt:
                return

        self.get_logger().info('Competition is ready. Starting...')

        # Check if service is available
        if not self._start_competition_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('Service \'/ariac/start_competition\' is not available.')
            return

        # Create trigger request and call starter service
        request = Trigger.Request()
        future = self._start_competition_client.call_async(request)

        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info('Started competition.')

            # get kit tray table information
            # self._get_kit_station_pose()
        else:
            self.get_logger().warn('Unable to start competition')
    
    def end_competition(self):
        '''
        Function to end the competition.
        '''
        # wait for all orders announced and all orders submitted
        if self._competition_state != CompetitionStateMsg.ORDER_ANNOUNCEMENTS_DONE or len(self._orders)>0:
            return

        self.get_logger().info('Ending the competition...');

        # Check if service is available
        if not self._end_competition_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('Service \'/ariac/end_competition\' is not available.')
            return

        # Create trigger request and call starter service
        request = Trigger.Request()
        future = self._end_competition_client.call_async(request)

        # Wait until the service call is completed
        # rclpy.spin_until_future_complete(self, future)
        future.add_done_callback(self.end_competition_done_cb)

    def end_competition_done_cb(self, future):
        '''
        Handle the future returned by service call to end competition
        '''
        if future.result().success:
            self.get_logger().info('Competition ended.')
        else:
            self.get_logger().warn('Unable to end competition')
    
    #Camera subscriber callbacks
    def _right_bins_camera_cb(self, msg: AdvancedLogicalCameraImageMsg):
        '''
        Arguments:
            msg -- AdvancedLogicalCameraImage message
        '''
        if "right_bins" in self._cam_updated:
            return
        self._camera_image = AdvancedLogicalCameraImage(msg.part_poses,
                                                        msg.tray_poses,
                                                        msg.sensor_pose)
        self.find_part_tray(self._camera_image,"right bins camera")
        self._cam_updated["right_bins"]=True
        
    def _left_bins_camera_cb(self, msg: AdvancedLogicalCameraImageMsg):
        '''
        Arguments:
            msg -- AdvancedLogicalCameraImage message
        '''
        if "left_bins" in self._cam_updated:
            return
        self._camera_image = AdvancedLogicalCameraImage(msg.part_poses,
                                                        msg.tray_poses,
                                                        msg.sensor_pose)
        self.find_part_tray(self._camera_image,"left bins camera")
        self._cam_updated["left_bins"]=True
        
    def _kts1_camera_cb(self, msg: AdvancedLogicalCameraImageMsg):
        '''
        Arguments:
            msg -- AdvancedLogicalCameraImage message
        '''
        if "kts1" in self._cam_updated:
            return
        self._camera_image = AdvancedLogicalCameraImage(msg.part_poses,
                                                        msg.tray_poses,
                                                        msg.sensor_pose)
        self.find_part_tray(self._camera_image,"kitting station 1 camera")
        self._cam_updated["kts1"]=True
        
    def _kts2_camera_cb(self, msg: AdvancedLogicalCameraImageMsg):
        '''
        Arguments:
            msg -- AdvancedLogicalCameraImage message
        '''
        if "kts2" in self._cam_updated:
            return
        self._camera_image = AdvancedLogicalCameraImage(msg.part_poses,
                                                        msg.tray_poses,
                                                        msg.sensor_pose)
        self.find_part_tray(self._camera_image,"kitting station 2 camera")
        self._cam_updated["kts2"]=True
    
    def find_part_tray(self, image: AdvancedLogicalCameraImage, camera_name="cam"):
        """find the trays and parts from the advanced logical camera image

        Args:
            image (AdvancedLogicalCameraImage): 
            camera_name (str, optional): name of the camera. Defaults to "cam".
        """

        if self._check_for_new_parts is False: # save bandwidth
            return

        # first look for parts
        for i, part_pose in enumerate(image._part_poses):
            part_pose: PartPoseMsg

            # convert pose to world frame
            part_pose.pose = multiply_pose(image._sensor_pose, part_pose.pose)

            # compare with parts stored in the list
            new_part = True
            for stored_part in self._parts:
                stored_part : Part
                if stored_part.color == part_pose.part.color and stored_part.type == part_pose.part.type and \
                    dist_two_poses(stored_part.pose, part_pose.pose) < 0.1:
                    # presumably same part, skip
                    new_part = False
                    break
            
            if new_part:
                self._part_count += 1
                self._parts.append(Part(part_msg=part_pose,part_id=self._part_count))
        
        # second look for trays
        for i, tray_pose in enumerate(image._tray_poses):
            tray_pose: KitTrayPose

            # convert pose to world frame
            tray_pose.pose = multiply_pose(image._sensor_pose, tray_pose.pose)

            # compare with parts stored in the list
            new_tray = True
            for stored_tray in self._trays:
                stored_tray : Tray
                if stored_tray.id == tray_pose.id and dist_two_poses(stored_tray.pose, tray_pose.pose) < 0.1:
                    # presumably same tray, skip
                    new_tray = False
                    break
            
            if new_tray:
                self._trays.append(Tray(tray_msg=tray_pose))

        # self.get_logger().info(f'Currently part number: {len(self._parts)}, tray number: {len(self._trays)}')

    def parse_advanced_camera_image(self, image: AdvancedLogicalCameraImage, camera_name="cam") -> str:
        '''
        Parse an AdvancedLogicalCameraImage message and return a string representation.
        '''
        
        if len(image._part_poses) == 0:
            return 'No parts detected'

        output = '\n\n'
        for i, part_pose in enumerate(image._part_poses):
            part_pose: PartPoseMsg
            output += '==========================\n'
            part_color = CompetitionInterface._part_colors[part_pose.part.color].capitalize()
            part_type = CompetitionInterface._part_types[part_pose.part.type].capitalize()
            output += f'{part_color} {part_type} detected by {camera_name}\n'
            output += '--------------------------\n'
            output += 'Camera Frame\n'
            output += '--------------------------\n'
            
            output += '  Position:\n'
            output += f'    x: {part_pose.pose.position.x:.3f} (m)\n'
            output += f'    y: {part_pose.pose.position.y:.3f} (m)\n'
            output += f'    z: {part_pose.pose.position.z:.3f} (m)\n'

            roll, pitch, yaw = rpy_from_quaternion(part_pose.pose.orientation)
            output += '  Orientation:\n'
            output += f'    roll: {rad_to_deg_str(roll)}\n'
            output += f'    pitch: {rad_to_deg_str(pitch)}\n'
            output += f'    yaw: {rad_to_deg_str(yaw)}\n'
            
            part_world_pose = multiply_pose(image._sensor_pose, part_pose.pose)
            output += '--------------------------\n'
            output += 'World Frame\n'
            output += '--------------------------\n'

            output += '  Position:\n'
            output += f'    x: {part_world_pose.position.x:.3f} (m)\n'
            output += f'    y: {part_world_pose.position.y:.3f} (m)\n'
            output += f'    z: {part_world_pose.position.z:.3f} (m)\n'

            roll, pitch, yaw = rpy_from_quaternion(part_world_pose.orientation)
            output += '  Orientation:\n'
            output += f'    roll: {rad_to_deg_str(roll)}\n'
            output += f'    pitch: {rad_to_deg_str(pitch)}\n'
            output += f'    yaw: {rad_to_deg_str(yaw)}\n'

            output += '==========================\n\n'
        
        # self.get_logger().info(output)

        return output
    
    def _orders_cb(self, msg: OrderMsg):
        """callback for subscription to the ariac order topic

        Args:
            msg (OrderMsg): order msg
        """
        order = msg
        self._orders.append(order)
        if self._parse_incoming_order:
            self.get_logger().info(self._parse_order(order))

    def _parse_order(self, order: OrderMsg):
        """parse incoming order msg

        Args:
            order (OrderMsg): 

        Returns:
            _type_: string of the parsed order
        """
        output = '\n\n==========================\n'
        output += f'Received Order: {order.id}\n'
        output += f'Priority: {order.priority}\n'

        if order.type == OrderMsg.KITTING:
            output += self._parse_kitting_task(order, order.kitting_task)
        else:
            output += 'Type: Unknown\n'
        return output
    
    def _parse_kitting_task(self, order:OrderMsg, kitting_task: KittingTask):
        """parse kitting task

        Args:
            order (OrderMsg): announced order msg
            kitting_task (KittingTask): kitting task msg

        Returns:
            _type_: string output
        """
        output = 'Type: Kitting\n'
        output += '==========================\n'
        output += f'AGV: {kitting_task.agv_number}\n'

        output += f'Destination: {CompetitionInterface._destinations[kitting_task.destination]}\n'
        output += f'Tray ID: {kitting_task.tray_id}\n'
        output += 'Products:\n'
        output += '==========================\n'

        quadrants = {1: "Quadrant 1: -",
                    2: "Quadrant 2: -",
                    3: "Quadrant 3: -",
                    4: "Quadrant 4: -"}

        for i in range(1, 5):
            product: KittingPart
            for product in kitting_task.parts:
                if i == product.quadrant:
                    part_color = CompetitionInterface._part_colors[product.part.color].capitalize()
                    part_type = CompetitionInterface._part_types[product.part.type].capitalize()
                    quadrants[i] = f'Quadrant {i}: {part_color} {part_type}'
        output += f'\t{quadrants[1]}\n'
        output += f'\t{quadrants[2]}\n'
        output += f'\t{quadrants[3]}\n'
        output += f'\t{quadrants[4]}\n'


        # self._tray_id.append(kitting_task.tray_id)
        # # Publish the message
        # agvnumber = UInt8()
        # agvnumber.data = kitting_task.agv_number
        # self._move_agv_pub.publish(agvnumber)

        # output += f'Published AGV number {agvnumber.data} to move_agv_warehouse'
        
        return output

    def _create_tree_cb(self):
        """builds the action tree for each announced order
        """

        # build the action tree for this task
        # first do a discovery of all parts
        # self._check_for_new_parts = True
        # time.sleep(5.0)
        # self._check_for_new_parts = False

        if self._all_cam_updated() is False:
            return

        if self._agv_pose_updated is False or len(self._trays)==0 or \
            len(self._parts)==0:
            return
        
        for order in self._orders:
            order : OrderMsg
            if order.id in self._action_trees:
                continue

            # for key in self._action_trees.keys():
            #     self.get_logger().info(f"Diction key {key}")
            # self.get_logger().info(f"Creating action tree for {order.id}...")
            # self.get_logger().info("Current part poses\n" + print_poses(self._parts))
            # self.get_logger().info("Current tray poses\n" + print_poses(self._trays))
            self.get_logger().info(f"++++++++++++{order.id}")
            for part in self._parts:
                part : Part
                self.get_logger().info(part.print())
            root : ActionNode = ActionTree.create_tree_from_kitting(order=order,
                                                                    kitting_task=order.kitting_task,
                                                                    parts=self._parts,
                                                                    trays=self._trays,
                                                                    agv_poses=self._agv_poses,
                                                                    kts_poses=self._kts_poses)
            
            self._action_trees[order.id] = root
            self._order_status[order.id] = self._ORDER_IN_PROCESS
            
            # self.get_logger().info(ActionTree.print_tree(root))

            # print("Assigned order for parts:")
            # print_assigned_order(self._parts)

            # print("Assigned order for trays:")
            # print_assigned_order(self._trays)

            # push the root onto the action queue
            self._action_queue.append(root)
            root.cost.calculate(root, self._floor_base, self._agv_poses, self._kts_poses,
                                self._curr_gripper, self._competition_state)
            # self._calculate_cost(root)

    def _manage_action_queue_cb(self):
        """manages enqueue and dequeue action nodes in the
        action queue
        """
        
        if len(self._action_queue) == 0:
            # self.get_logger().info("Action queue empty.")
            return
        
        # self.get_logger().info(f"Current action queue length: {len(self._action_queue)}")

        # get the front
        front : ActionNode = self._action_queue[0]
        
        front.is_completed : List[bool]
        # self.get_logger().info(f"{str(front.is_completed)}")
        if False not in front.is_completed:
            self.get_logger().info("🏁 Current action completed!")
            # remove node
            self._action_queue.pop(0)
            # enqueue next actions
            for next_action in front.next_actions:
                self.get_logger().info("➡️ Enqueuing new action...")
                self._action_queue.append(next_action)

            # (re)compute all action costs and reorder the queue
            self._update_cost()
            self._sort_queue()
            
        else:
            front.service_call_sent : List[bool]
            # self.get_logger().info(f"Current action service call sent {sum(front.service_call_sent)} / {len(front.service_call_sent)}")
            if False in front.service_call_sent: # not all service call sent within the node
                # send service call
                self._send_service_call(front)
                return
            else: # already sent, wait for result
                pass

        return
    
    def _send_service_call(self, node : ActionNode):
        """send service calls to complete required action

        Args:
            node (ActionNode): _description_
        """
        # change gripper
        if node.action == ActionNode.ACTION_CHANGE_GRIPPER:
            self._call_change_gripper_service(node)
            return
            
        # pick and place tray
        # self.get_logger().info(f"Current action object type: {ActionNode.OBJECT_TYPE_TRAY}");
        if node.action == ActionNode.ACTION_PICK_PLACE and node.object_type == ActionNode.OBJECT_TYPE_TRAY:
            if node.service_call_sent[0] is False:
                # move to table
                self._call_move_to_table_srv(node)
                return
            
            if node.is_completed[0] is True and node.service_call_sent[1] is False:
                # send pick service call
                self._activate_gripper()
                self._call_pickup_tray_service(node)
                return

            if node.is_completed[1] is True and node.service_call_sent[2] is False:
                # send place service call if pick is sent and completed
                self._call_place_tray_service(node)
                return
            
            return

        # pick and place part
        if node.action == ActionNode.ACTION_PICK_PLACE and node.object_type == ActionNode.OBJECT_TYPE_PART:
            if node.part is None:
                self._handle_insufficient_part(node)

            if node.service_call_sent[0] is False:
                # send pick service call
                self.get_logger().debug('is this getting executed')
                self._activate_gripper()
                self._call_pickup_part_service(node)
                return
            
            if node.is_completed[0] is True and node.service_call_sent[1] is False:
                # send place service call if pick is sent and completed
                self._call_place_part_service(node)
                return
            
            if node.is_completed[1] is True and node.service_call_sent[2] is False:
                # send check quality service
                self._call_quality_check(node)
                return
            return
        
        # remove faulty part
        if node.action == ActionNode.ACTION_REMOVE_PART_FROM_AGV:
            if node.service_call_sent[0] is False:
                self._activate_gripper()
                self._call_remove_part_service(node)

        # if all actions complete, return robot to home base
        if False not in node.is_completed:
            self._call_move_robot_home_service(node)
            return

    
    def _handle_insufficient_part(self, node : ActionNode):
        # insufficient part challenge
        # If all orders have been announced, do the following in order:
        # 1. check whether matching part exists (double check).
        part_found = ActionTree.find_part(order_id=node.order_id,
                                        parts=self._parts,
                                        target_color=node.target_part_color,
                                        target_type=node.target_part_type,
                                        target_pose=self._agv_poses[node.agv_number-1])
        
        if part_found is not None: # part is found, use it
            part_found.quadrant = node.target_quadrant
            node.add_part(part_found)
            return

        # part_found is None, i.e., no matching part is found
        # first check if all orders have been announced
        if self._competition_state == CompetitionStateMsg.ORDER_ANNOUNCEMENTS_DONE:
            # if all orders announced, check if part of a different color is available
            for color in CompetitionInterface._part_colors:
                if color == node.target_part_color:
                    continue

                # use the part if found
                part_found = ActionTree.find_part(order_id=node.order_id,
                                    parts=self._parts,
                                    target_color=color,
                                    target_type=node.target_part_type,
                                    target_pose=self._agv_poses[node.agv_number-1])
                if part_found is not None:
                    part_found : Part
                    part_found.quadrant = node.target_quadrant
                    node.add_part(part_found)
                    break
                
            if node.part is None: # none found for all other colors
                node.mark_complete_all() # no part of the type available, mark the action as done
                return
            else:
                pass # part of other color is found
    
        else: # not all orders have been announced, do nothing
            return

    def _call_move_to_table_srv(self, node : ActionNode):
        """call service to move the robot to the kts table

        Args:
            node (ActionNode): _description_
        """
        if dist_two_poses(self._floor_base, self._kts_poses[node.kts_table_no-1]) < 2.0:
            node.service_call_sent[node.ind_service_call] = True
            # node.is_completed[node.ind_service_call] = True
            # node.ind_service_call += 1
            node.mark_curr_service_done()
            return
        
        while not self._move_robot_to_table_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Change gripper service not available, waiting...')

        request = MoveRobotToTable.Request()
        request.kts = node.kts_table_no

        # send request
        future = self._move_robot_to_table_cli.call_async(request)
        node.service_call_sent[node.ind_service_call] = True

        # Add a callback function to be called when the future is complete
        future.add_done_callback(self._move_to_table_done_cb)

    def _move_to_table_done_cb(self, future : Future):
        curr_action : ActionNode = self._action_queue[0]
        if future.result().success is True:
            curr_action.mark_curr_service_done()
            # curr_action.is_completed[curr_action.ind_service_call] = True
            # curr_action.ind_service_call += 1
            self.get_logger().info(f"Moved to the table!")
        else:
            self.get_logger().fatal(f"FAILED to move to the table")
        return


    def _call_change_gripper_service(self, node : ActionNode):
        """call change gripper service

        Args:
            node (ActionNode): the detail of the call is in the node
        """
        # check if the current gripper is already the one requested
        if (node.object_type == ActionNode.OBJECT_TYPE_TRAY and \
            self._curr_gripper == AriacChangeGripper.Request.TRAY_GRIPPER) or \
            (node.object_type == ActionNode.OBJECT_TYPE_PART and \
            self._curr_gripper == AriacChangeGripper.Request.PART_GRIPPER):

            # mark as completed
            curr_action : ActionNode = self._action_queue[0]
            # curr_action.is_completed[0] = True
            curr_action.mark_curr_service_done()
            return

        # self.get_logger().info(f"Sending change gripper service call for order {node.order_id}")
            # send change gripper ser vice call
        request = CustomChangeGripper.Request()

        if node.object_type == ActionNode.OBJECT_TYPE_TRAY: # tray gripper
            request.gripper_type = CustomChangeGripper.Request.TRAY_GRIPPER
        else:
            request.gripper_type = CustomChangeGripper.Request.PART_GRIPPER
            # update kts number based on the robot position, goes to the closest
            # kts to change gripper
            parts_poses = []
            for n_node in node.next_actions:
                if n_node.part_pose is not None:
                    parts_poses.append(n_node.part_pose)

            if len(parts_poses)>0:
                ind = min_sum_pose_dist(poses=self._kts_poses,
                                        target_poses=parts_poses)
            else:
                ind = min_ind_pose_dist(poses=self._kts_poses,
                                    target_pose=self._floor_base)
            
            node.kts_table_no = ind+1

        if node.kts_table_no == 1: # table 1
            request.table = CustomChangeGripper.Request.TABLE1
        elif node.kts_table_no == 2: # table 2
            request.table = CustomChangeGripper.Request.TABLE2

        # use floor robot by default
        request.robot = CustomChangeGripper.Request.FLOOR_ROBOT

        while not self._change_gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Change gripper service not available, waiting...')

        # send request
        future = self._change_gripper_client.call_async(request)
        node.service_call_sent[node.ind_service_call] = True

        # Add a callback function to be called when the future is complete
        future.add_done_callback(self._change_gripper_future_cb)

    def _activate_gripper(self):
        '''
        Activate the gripper
        '''
        self.get_logger().info('👉 Activating gripper...')
        self._activating_gripper = True
        while not self._set_gripper_state_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        request = VacuumGripperControl.Request()
        request.enable = True
        future = self._set_gripper_state_cli.call_async(request)
        future.add_done_callback(self._activate_gripper_done_cb)

    def _activate_gripper_done_cb(self, future):
        '''
        Client callback for the service /ariac/floor_robot_enable_gripper

        Args:
            future (Future): A future object
        '''
        if future.result().success:
            self.get_logger().info('✅ Gripper activated')
            self._activated_gripper = True
        else:
            self.get_logger().fatal('💀 Gripper not activated')

    def _deactivate_gripper(self):
        '''
        Deactivate the gripper
        '''
        self.get_logger().info('👉 Deactivating gripper...')
        self._deactivating_gripper = True
        while not self._set_gripper_state_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        request = VacuumGripperControl.Request()
        request.enable = False
        future = self._set_gripper_state_cli.call_async(request)
        future.add_done_callback(self._deactivate_gripper_done_cb)

    def _deactivate_gripper_done_cb(self, future):
        '''
        Client callback for the service /ariac/floor_robot_enable_gripper

        Args:
            future (Future): A future object
        '''
        if future.result().success:
            self.get_logger().info('✅ Gripper deactivated')
            self._deactivated_gripper = True
        else:
            self.get_logger().fatal('💀 Gripper not deactivated')


    def _call_pickup_tray_service(self,node: ActionNode):
        """call pick up tray service according to node specification

        Args:
            node (ActionNode): 
        """
        # self.get_logger().info(f"Sending pick tray service call for order {node.order_id}")
        # send pick part service call

        request = MoveRobotToTray.Request()
        # build tray_id and tray_pose request
        request.tray_id = node.tray_id
        request.tray_pose_in_world = node.tray_pose

        # request.robot = PickupTray.Request.FLOOR_ROBOT  # use floor robot by default
                
                
        while not self._pickup_tray_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Pick tray service not available, waiting...')

         # send request
        future = self._pickup_tray_client.call_async(request)
        node.service_call_sent[node.ind_service_call] = True

        # Add a callback function to be called when the future is complete
        future.add_done_callback(self._pickup_tray_client_callback)

    
    def _call_pickup_part_service(self,node: ActionNode):
        """call pick up part service according to node specification

        Args:
            node (ActionNode): 
        """
        # self.get_logger().info(f"Sending pick part service call for order {node.order_id}")
        # send pick part service call
        request = PickupPart.Request()
        # populate the requests
        request.robot = PickupPart.Request.FLOOR_ROBOT
        request.part_color = node.part_color
        request.part_type = node.part_type
        request.part_pose = node.part_pose  # The pose of the part

        while not self._pickup_part_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Pick part service not available, waiting...')

        # send request
        future = self._pickup_part_client.call_async(request)
        node.service_call_sent[node.ind_service_call] = True

        # function to be called when the future is complete
        future.add_done_callback(self._pickup_part_client_callback)

    def _call_place_tray_service(self,node: ActionNode):
        """call place tray service according to node specification

        Args:
            node (ActionNode): 
        """
        # self.get_logger().info(f"Sending place tray service call for order {node.order_id}")
        # send pick part service call

        request = PlacingTray.Request()
        # build tray_id and tray_pose request
        request.tray_id = node.tray_id
        request.agv_id = node.agv_number

        while not self._place_tray_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Place tray service not available, waiting...')

         # send request
        future = self._place_tray_client.call_async(request)
        node.service_call_sent[node.ind_service_call] = True # index 0 is for pick task

        # Add a callback function to be called when the future is complete
        future.add_done_callback(self._place_tray_client_callback)

    def _call_place_part_service(self,node: ActionNode):
        """call place part service according to node specification

        Args:
            node (ActionNode): _description_
        """
        self.get_logger().info(f"Sending placing part service call for order {node.order_id}")
        # send pick part service call
        request = PlacingPart.Request()
        # populate the requests
        request.robot = PlacingPart.Request.FLOOR_ROBOT
        request.part_color = node.part_color
        request.part_type = node.part_type
        request.tray_id = node.tray_id
        request.agv_id = node.agv_number
        request.quadrant_id = node.part_quadrant
            
        while not self._place_part_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Placing part service not available, waiting...')

        # send request
        future = self._place_part_client.call_async(request)
        node.service_call_sent[node.ind_service_call] = True # index 0 is for pick task

        # function to be called when the future is complete
        future.add_done_callback(self._place_part_client_callback)

    def _call_move_robot_home_service(self,node: ActionNode):
        """call place part service according to node specification

        Args:
            node (ActionNode): _description_
        """
        self.get_logger().info(f"Sending floor robot home service call for order {node.order_id}")
        # send pick part service call
        request = Trigger.Request()
        
            
        while not self._move_robot_to_home_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Moving robot to home service not available, waiting...')

        # send request
        future = self._move_robot_to_home_cli.call_async(request)
        node.service_call_sent[0] = True 

        # function to be called when the future is complete
        future.add_done_callback(self._move_robot_home_client_callback)

    def _change_gripper_future_cb(self, future : Future):
        """handles change gripper service call result

        Args:
            future (Future): _description_
        """
        curr_action : ActionNode = self._action_queue[0]
        if future.result().success is True:
            # curr_action.is_completed[0] = True
            curr_action.mark_curr_service_done()
            # update gripper status
            self._curr_gripper = AriacChangeGripper.Request.TRAY_GRIPPER \
                if curr_action.object_type == ActionNode.OBJECT_TYPE_TRAY else AriacChangeGripper.Request.PART_GRIPPER
            
            # self.get_logger().info(f"Change gripper service call successful for {curr_action.order_id}")
        else:
            self.get_logger().fatal(f"Change gripper service call failed for {curr_action.order_id}")
        return
    
    def _pickup_tray_client_callback(self, future: Future):
        """handles pick up tray service call result

        Args:
            future (Future): _description_
        """
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error('Service call failed: %r' % (e,))
        else:
            if response.success:
                curr_action : ActionNode = self._action_queue[0]
                # curr_action.is_completed[1] = True # pick is successful, place is not yet
                curr_action.mark_curr_service_done()
                # self.get_logger().info('Successfully picked up the tray.')
            else:
                self.get_logger().fatal('Failed to pick up the tray.')

    def _pickup_part_client_callback(self, future: Future):
        """handles pick up part service call result

        Args:
            future (Future): _description_
        """
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().info('Service call failed %r' % (e,))
        else:
            if response.success:
                curr_action : ActionNode = self._action_queue[0]
                # curr_action.is_completed[0] = True # pick is successful, place is not yet
                curr_action.mark_curr_service_done()
                self.get_logger().info('Pickup was successful')
                # Store this data or use it for further action
                # self._parts.append({
                #     "part_type": self._part_types[response.part_type],
                #     "part_color": self._part_colors[response.part_color],
                #     "part_pose": response.part_pose
                # })
            else:
                self.get_logger().fatal('Failed to pick up the part.')

    def _place_tray_client_callback(self, future: Future):
        """handles place tray service call result

        Args:
            future (Future): _description_
        """
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error('Service call failed: %r' % (e,))
        else:
            if response.success:
                curr_action : ActionNode = self._action_queue[0]
                # curr_action.is_completed[2] = True # both pick and place should be successful now
                curr_action.mark_curr_service_done()
                # self._deactivate_gripper()
                self.get_logger().info('📥 Successfully placed the tray.')
            else:
                self.get_logger().fatal('Failed to place the tray.')

    def _place_part_client_callback(self, future: Future):
        """handles place part service call result

        Args:
            future (Future): _description_
        """
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().info('Service call failed %r' % (e,))
        else:
            if response.success:
                curr_action : ActionNode = self._action_queue[0]
                # curr_action.is_completed[1] = True # both pick and place should be successful now
                curr_action.mark_curr_service_done()
                # self.get_logger().info('Placing part was successful')

            else:
                self.get_logger().fatal('Failed to place the part.')

    def _move_robot_home_client_callback(self, future: Future):
        """handles move robot home service call result

        Args:
            future (Future): _description_
        """
        curr_action : ActionNode = self._action_queue[0]
        if future.result().success is True:
            curr_action.is_completed[0] = True
            self.get_logger().info(f"Moved to the table!")
        else:
            self.get_logger().fatal(f"FAILED to move to the table")
        return

    def _calculate_cost(self, node : ActionNode):
        """calculate the cost for the current node

        Args:
            node (ActionNode): _description_
        """
        if self._floor_base_updated is False:
            return
        
        if node.action == ActionNode.ACTION_CHANGE_GRIPPER:
            # find distance to the kit station table
            table_ind = node.kts_table_no # 1 or 2
            _dist = dist_two_poses(self._floor_base, self._kts_poses[table_ind-1])

            # change gripper has additional cost associated
            if (node.object_type == ActionNode.OBJECT_TYPE_TRAY and \
                self._curr_gripper == AriacChangeGripper.Request.TRAY_GRIPPER) or \
                (node.object_type == ActionNode.OBJECT_TYPE_PART and \
                self._curr_gripper == AriacChangeGripper.Request.PART_GRIPPER):
                # if the current gripper matches the one requested by the action
                _chg_cost = 0.0
            else:
                _chg_cost = 1.0

            node.cost = self._k_cost_dist*_dist + self._k_cost_chg_gripper*_chg_cost

        elif node.action == ActionNode.ACTION_PICK_PLACE:
            if self._agv_pose_updated is False:
                return
            
            if node.object_type == ActionNode.OBJECT_TYPE_PART:
                object_pose = node.part_pose
            else:
                object_pose = node.tray_pose

            # find distance to part/tray (robot goes to the object)
            _dist1 = dist_two_poses(self._floor_base, object_pose)
            # robot then goes to the agv from the object
            _dist2 = dist_two_poses(object_pose, self._agv_poses[node.agv_number-1])

            # check if the gripper mismatches
            if (node.object_type == ActionNode.OBJECT_TYPE_TRAY and \
                self._curr_gripper == AriacChangeGripper.Request.TRAY_GRIPPER) or \
                (node.object_type == ActionNode.OBJECT_TYPE_PART and \
                self._curr_gripper == AriacChangeGripper.Request.PART_GRIPPER):
                _gripper_mismatch_cost = 0.0
            else:
                _gripper_mismatch_cost = 1.0

            node.cost = self._k_cost_dist*(_dist1 + _dist2) + self._k_cost_chg_gripper * _gripper_mismatch_cost
        return

    def _update_cost(self):
        """updates the cost for each node in the action queue

        """
        for node in self._action_queue:
            # self._calculate_cost(node)
            node : ActionNode
            node.cost.calculate(node, self._floor_base, self._agv_poses, self._kts_poses,
                                self._curr_gripper, self._competition_state)
                       

    def _sort_queue(self):
        """sort the action queue according to cost of performing the action

        """
        self._action_queue.sort(key=lambda x: x.cost.value)

    def _competition_state_cb(self, msg: CompetitionStateMsg):
        '''Callback for the topic /ariac/competition_state

        Arguments:
            msg -- CompetitionState message
        '''
        # Log if competition state has changed
        if self._competition_state != msg.competition_state:
            state = CompetitionInterface._competition_states[msg.competition_state]
            self.get_logger().info(f'Competition state is: {state}', throttle_duration_sec=1.0)
        
        self._competition_state = msg.competition_state                      
    
    def submitted_order_sub_cb(self, msg: String):
        '''
        Callback for subscribing to '/ariac/submitted_order' topic
        '''

        # remove submitted order
        for (i, order) in enumerate(self._orders):
            if order.id == msg.data:
                self._orders.pop(i)
                break
        
        # try end the competition
        if len(self._orders)==0: # all orders submitted
            self.end_competition()
    
    def _check_order_status_cb(self):
        """for each action tree, check if all actions have been performed
        and move on to the next status
        """
        for (order_id, root) in self._action_trees.items():
            root : ActionNode
            if self._order_status[order_id] == self._ORDER_IN_PROCESS:
                if self._check_order_complete(root) is True:
                    self._order_status[order_id] = self._ORDER_COMPLETED
                    self.get_logger().info(f"✅✅✅✅ Order {order_id} finished! ✅✅✅✅")
            elif self._order_status[order_id] == self._ORDER_COMPLETED:
                # ship the order
                agvnumber = UInt8()
                agvnumber.data = root.agv_number
                self._move_agv_pub.publish(agvnumber)
                self._order_status[order_id] = self._ORDER_SHIPPED
                self.get_logger().info(f"🚚🚚🚚🚚 Order {order_id} shipped! 🚚🚚🚚🚚")

    def _check_order_complete(self, node : ActionNode):
        """check if the current node and all children nodes have completed

        Args:
            node (ActionNode): 

        Returns:
            _type_: True of False
        """
        all_completed = True
        if False not in node.is_completed: # this action node is completed
            for next_action in node.next_actions:
                all_completed = all_completed & self._check_order_complete(next_action)
        else:
            all_completed = False
        return all_completed
    
    def _get_kit_station_pose(self):
        """find the poses of the two kitting stations
        """
        for i, tf_buffer in enumerate(self._tf_buffer_kts):
            try:
                transform = tf_buffer.lookup_transform(
                    'world', f'kts{i+1}_table_frame', rclpy.time.Time())
                
                self._kts_poses[i].position.x = transform.transform.translation.x
                self._kts_poses[i].position.y = transform.transform.translation.y
                self._kts_poses[i].position.z = transform.transform.translation.z

                self._kts_poses[i].orientation = transform.transform.rotation

            except:
                self.get_logger().fatal(
                    f"Could not get transform between world and f'kts{i+1}_table_frame'")
        return

    def _listener_floor_robot_cb(self):
        """updates the floor robot's base pose
        """
        try:
            transform = self._tf_buffer_floor_robot.lookup_transform(
                'world', 'floor_base', rclpy.time.Time())
            
            self._floor_base.position.x = transform.transform.translation.x
            self._floor_base.position.y = transform.transform.translation.y
            self._floor_base.position.z = transform.transform.translation.z

            self._floor_base.orientation = transform.transform.rotation
            self._floor_base_updated = True
        except:
            self.get_logger().fatal(
                    f"Could not get transform between world and floor_base")
            
    def _agv_pose_listner_cb(self):
        '''
        Callback function for the listener timer.
        '''
        
        # Get the transform between frames
        n_updated = 0
        for i, tf_buffer in enumerate(self._tf_buffer_agv):
            agv_base = f'agv{i+1}_base'
            try:
                transform = tf_buffer.lookup_transform(
                    'world', agv_base, rclpy.time.Time())
                
                self._agv_poses[i].position.x = transform.transform.translation.x
                self._agv_poses[i].position.y = transform.transform.translation.y
                self._agv_poses[i].position.z = transform.transform.translation.z

                self._agv_poses[i].orientation = transform.transform.rotation
                n_updated += 1

                # self.get_logger().info(
                #     f"{agv_base} coordinate: {transform.transform.translation.x},\
                #         {transform.transform.translation.y}, {transform.transform.translation.z}: \n")
            except:
                self.get_logger().fatal(
                    f"Could not get transform between world and {agv_base}")

            if n_updated == 4:
                self._agv_pose_updated = True

    def _call_quality_check(self, node:ActionNode):
        """send service call to perform quality check on parts put on the gv

        Args:
            order_id (str): _description_
            quadrant_id (int): _description_

        Returns:
            _type_: _description_
        """
        req = PerformQualityCheck.Request()
        req.order_id = node.order_id

        future = self._quality_check_cli.call_async(req)
        future.add_done_callback(self._quality_check_done_cb)
        node.service_call_sent[node.ind_service_call] = True
    
    def _quality_check_done_cb(self, future : Future):
        """handles the result of quality check 

        Args:
            future (_type_): _description_

        Returns:
            _type_: _description_
        """

        curr_action : ActionNode = self._action_queue[0]
        quadrant_id = curr_action.part_quadrant

        if quadrant_id == 1:
            is_faulty = self._check_faulty(future.result().quadrant1)
        elif quadrant_id == 2:
            is_faulty = self._check_faulty(future.result().quadrant2)
        elif quadrant_id == 3:
            is_faulty = self._check_faulty(future.result().quadrant3)
        elif quadrant_id == 4:
            is_faulty = self._check_faulty(future.result().quadrant4)

        faulty_str = "faulty ❌." if is_faulty else "not faulty ✅."
        self.get_logger().info(f"Part on quadrant {quadrant_id} of AGV {curr_action.agv_number} is {faulty_str}")

        if is_faulty is False:
            # curr_action.is_completed[2] = True # mark entire action node as complete
            curr_action.mark_curr_service_done()
        else:
            for part in self._parts:
                self.get_logger().info(part.print())
            report = ActionTree.insert_remove_part_node(curr_action=curr_action,
                                               parts=self._parts,
                                               agv_pose=self._agv_poses[curr_action.agv_number-1])
            self.get_logger().info(report)
            
    def _check_faulty(self, result: QualityIssue):
        """check if the quality check detects a fault part

        Args:
            result (QualityIssue): _description_

        Returns:
            _type_: _description_
        """
        return result.faulty_part
        
    def _call_remove_part_service(self, node:ActionNode):
        """send service call to remove part from AGV

        Args:
            node (ActionNode): _description_
        """

        while not self._remove_part_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Remove part service not available, waiting...')

        req = RemovePart.Request()
        req.agv_id = node.agv_number
        req.quadrant_id = node.part_quadrant
        req.part_type = node.part_type
        req.part_color = node.part_color

        future = self._remove_part_client.call_async(req)
        node.service_call_sent[node.ind_service_call] = True
        future.add_done_callback(self._remove_part_done_cb)
    
    def _remove_part_done_cb(self, future : Future):
        """handle the result of remove part service call

        Args:
            future (Future): _description_
        """
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error('Service call failed: %r' % (e,))
        else:
            if response.success:
                curr_action : ActionNode = self._action_queue[0]
                # curr_action.is_completed[curr_action.ind_service_call] = True
                # curr_action.ind_service_call += 1
                curr_action.mark_curr_service_done()

                self.get_logger().info('📤 Successfully removed the part.')
            else:
                self.get_logger().fatal('Failed to remove part.')
    
    def _all_cam_updated(self):
        cam_names = ["right_bins","left_bins","kts1","kts2"]

        for cam_name in cam_names:
            if cam_name not in self._cam_updated:
                return False
        
        return True