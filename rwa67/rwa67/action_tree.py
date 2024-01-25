import numpy as np
from typing import List
import copy

from ariac_msgs.msg import (
    PartPose,
    Order,
    KittingTask,
    KittingPart,
    KitTrayPose,
    CompetitionState as CompetitionStateMsg
)

from ariac_msgs.srv import (
    ChangeGripper as AriacChangeGripper
)

from geometry_msgs.msg import TransformStamped, Pose, Quaternion

from rwa67.utils import (
    multiply_pose,
    rpy_from_quaternion,
    rad_to_deg_str,
    AdvancedLogicalCameraImage,
    dist_two_poses,
    min_ind_pose_dist,
    Part,
    Tray
)

class ActionNode:

    # define some constants for the actions
    FLOOR_ROBOT = 0
    CEILING_ROBOT = 1
    
    ACTION_CHANGE_GRIPPER = 0
    ACTION_PICK_PLACE = 1
    ACTION_REMOVE_PART_FROM_AGV = 2
    
    OBJECT_TYPE_TRAY = 0
    OBJECT_TYPE_PART = 1

        # when change gripper, the object_description will be used to specify
    # the table the robot should go to
    TRAY_TABLE_NO = [1,2]

    def __init__(self,
                 order_id: str,
                 robot: int,
                 action: int,
                 object_type: int,
                 next_actions: list = None,
                 agv_number = None,
                 part = None,
                 tray = None,
                 kts_table_no = None,
                 priority = False):
        """create new action node for action tree

        Args:
            order_id (str): order id
            robot (int): floor or ceiling robot
            action (int): change gripper or pick and place
            object_type (int): tray or part
            action_cost (float, optional): estimated cost to perform the action right now. Defaults to 0.0.
            next_actions (list, optional): children action nodes. Defaults to [].
            agv_number (optional): the agv number for the kitting task
            destination (optional): only for pick and place part, indicates [tray id, quadrant]
        """
        
        self.order_id = order_id
        self.robot = robot
        self.action = action
        self.object_type = object_type
        self.object_pose = Pose()
        self.agv_number = agv_number if agv_number else None
        self.cost = ActionCost()
        self.next_actions = next_actions if next_actions else []
        self._part : List[Part] = [part] if part else []
        self._tray : List[Tray] = [tray] if tray else []
        self.kts_table_no = kts_table_no
        self.priority = priority
        self.target_part_color = None
        self.target_part_type = None
        self.target_quadrant = None

        # for pick and place we need 2 conditions for pick and place separately
        if action==ActionNode.ACTION_CHANGE_GRIPPER:
            self.n_service_call = 1 # single custom gripper change call
        elif action==ActionNode.ACTION_PICK_PLACE and object_type==ActionNode.OBJECT_TYPE_TRAY:
            self.n_service_call = 3 # move to table / pick / place
        elif action==ActionNode.ACTION_PICK_PLACE and object_type==ActionNode.OBJECT_TYPE_PART:
            self.n_service_call = 3 # pick / place / quality check
        elif action==ActionNode.ACTION_REMOVE_PART_FROM_AGV:
            self.n_service_call = 1 # single custom service call

        self.service_call_sent : List[bool] = [False]*self.n_service_call
        self.is_completed : List[bool] = [False]*self.n_service_call
        self.ind_service_call = 0

    @property
    def tray(self):
        return None if len(self._tray) == 0 else self._tray[0]
    
    @property
    def part(self):
        return None if len(self._part) == 0 else self._part[0]
    
    @property
    def tray_pose(self):
        if len(self._tray) > 0:
            return self._tray[0].pose
        else:
            return None
    
    @property
    def part_pose(self):
        if len(self._part) > 0:
            return self._part[0].pose
        else:
            return None
    
    @property
    def part_color(self):
        if len(self._part) > 0:
            return self._part[0].color
        else:
            return None
        
    @property
    def part_type(self):
        if len(self._part) > 0:
            return self._part[0].type
        else:
            return None
    
    @property
    def part_quadrant(self):
        if len(self._part) > 0:
            return self._part[0].quadrant
        else:
            return None
        
    @property
    def part_id(self):
        if len(self._part) > 0:
            return self._part[0].id
        else:
            return None
        
    @property
    def tray_id(self):
        if len(self._tray) > 0:
            return self._tray[0].id
        else:
            return None
        
    def add_part(self, part : Part):
        """set the part of the current node

        Args:
            part (Part): _description_
        """
        self._part = [part]

    def reset_service_calls(self, n_service_call = None):
        """reset the service call sent member variable

        Args:
            n_service_call (_type_, optional): _description_. Defaults to None.
        """
        n = self.n_service_call if n_service_call is None else n_service_call
        self.service_call_sent = [False]*n
        self.is_completed = [False]*n
        self.ind_service_call = 0

    def mark_complete_all(self):
        """mark all service calls as complete

        Returns:
            _type_: _description_
        """
        self.service_call_sent = [True]*self.n_service_call
        self.is_completed = [True]*self.n_service_call

    def mark_curr_service_done(self):
        """mark the current service call as completed
        """
        self.is_completed[self.ind_service_call] = True
        self.ind_service_call += 1

    def reset_part(self):
        """reset the part to an empty list

        Returns:
            _type_: _description_
        """
        self._part : List[Part] = []

class ActionCost:
    cost_coef = dict()
    cost_coef["distance"] = 1.0
    cost_coef["change_gripper"] = 100.0
    cost_coef["insufficient_part"] = 200.0
    cost_coef["priority"] = 500.0

    def __init__(self):
        self.value = 0.0
    
    def calculate(self,
                node : ActionNode,
                robot_pose : Pose,
                agv_poses : List[Pose],
                kts_poses : List[Pose],
                curr_gripper,
                competition_state
                ):
        cost = dict()
        cost["distance"] = self._dist_cost(node, robot_pose, agv_poses, kts_poses)
        cost["change_gripper"] = self._gripper_cost(node, curr_gripper)
        cost["insufficient_part"] = self._insufficient_part_cost(node,competition_state)
        cost["priority"] = self._priority_cost(node)

        cost_sum = 0.0
        for key, val in cost.items():
            cost_sum += ActionCost.cost_coef[key] * val
            
        self.value = cost_sum
    
    def _dist_cost(self,
                    node : ActionNode,
                    robot_pose : Pose,
                    agv_poses : List[Pose],
                    kts_poses : List[Pose]):
        
        if node.action == ActionNode.ACTION_CHANGE_GRIPPER:
            table_ind = node.kts_table_no # 1 or 2
            _dist = dist_two_poses(robot_pose, kts_poses[table_ind-1])

        elif node.action == ActionNode.ACTION_PICK_PLACE:
            if node.object_type == ActionNode.OBJECT_TYPE_PART:
                object_pose = node.part_pose
            else:
                object_pose = node.tray_pose

            if object_pose is None:
                return 0.0
            # find distance to part/tray (robot goes to the object)
            _dist1 = dist_two_poses(robot_pose, object_pose)
            # robot then goes to the agv from the object
            _dist2 = dist_two_poses(object_pose, agv_poses[node.agv_number-1])

            _dist = _dist1 + _dist2
        
        elif node.action == ActionNode.ACTION_REMOVE_PART_FROM_AGV:
            _dist = 0.0

        return _dist
    
    def _gripper_cost(self, node: ActionNode, curr_gripper):
        if (node.object_type == ActionNode.OBJECT_TYPE_TRAY and \
            curr_gripper == AriacChangeGripper.Request.TRAY_GRIPPER) or \
            (node.object_type == ActionNode.OBJECT_TYPE_PART and \
            curr_gripper == AriacChangeGripper.Request.PART_GRIPPER):
            # if the current gripper matches the one requested by the action
            _gripper_cost = 0.0
        else:
            _gripper_cost = 1.0

        return _gripper_cost
    
    def _insufficient_part_cost(self, node : ActionNode, competition_state):
        if node.action == ActionNode.ACTION_PICK_PLACE and \
            node.object_type == ActionNode.OBJECT_TYPE_PART and \
            node.part is None:
                # if not all orders are announced, punish the node by pushing it to the back
                # if all orders are announced, no need to push to the back
                return 1.0 if competition_state != CompetitionStateMsg.ORDER_ANNOUNCEMENTS_DONE else 0.0
        return 0
        
    def _priority_cost(self, node : ActionNode):
        return 0.0 if node.priority is True else 1.0

class ActionTree:
    def __init__(self):
        return
    
    @staticmethod
    def create_tree_from_kitting(order: Order,
                                 kitting_task: KittingTask,
                                 parts : List[Part],
                                 trays : List[Tray],
                                 agv_poses : List[Pose],
                                 kts_poses : List[Pose]):
        """create an action tree from the kitting task

        Args:
            order (Order): an Order msg
            kittingTask (KittingTask): a KittingTask msg
        """

        # find which tray is the closest to the target AGV and
        # matches the tray id
        sub_ind = min_ind_pose_dist(poses=[tray.pose for tray in trays \
                                       if tray.id==kitting_task.tray_id and \
                                        tray.assigned_order == ""],
                                target_pose=agv_poses[kitting_task.agv_number-1])
        
        if sub_ind is not None:
            # figure out the correct indexing
            tray_ind = [i for i in range(len(trays)) if trays[i].id==kitting_task.tray_id \
                   and trays[i].assigned_order == ""][sub_ind]
            trays[tray_ind].assigned_order = order.id
            target_tray = trays[tray_ind]
            target_tray_pose = trays[tray_ind].pose

        # change to tray gripper
        # first find which table is closer to the target tray
        ind = min_ind_pose_dist(poses=kts_poses,
                                target_pose=target_tray_pose)
        kts_table_no = ActionNode.TRAY_TABLE_NO[ind]
        node1 = ActionNode(order_id=order.id,
                           robot=ActionNode.FLOOR_ROBOT,
                           action=ActionNode.ACTION_CHANGE_GRIPPER,
                           object_type=ActionNode.OBJECT_TYPE_TRAY,
                           kts_table_no=kts_table_no,
                           agv_number=kitting_task.agv_number)
        
        # pick and place tray
        node2 = ActionNode(order_id=order.id,
                           robot=ActionNode.FLOOR_ROBOT,
                           action=ActionNode.ACTION_PICK_PLACE,
                           object_type=ActionNode.OBJECT_TYPE_TRAY,
                           tray=target_tray,
                           kts_table_no=kts_table_no,
                           agv_number=kitting_task.agv_number)
        
        # give node2 the correct tray pose
        node2.object_pose = target_tray_pose
        
        # change to part gripper
        node3 = ActionNode(order_id=order.id,
                           robot=ActionNode.FLOOR_ROBOT,
                           action=ActionNode.ACTION_CHANGE_GRIPPER,
                           object_type=ActionNode.OBJECT_TYPE_PART,
                           kts_table_no=kts_table_no,
                           agv_number=kitting_task.agv_number)

        # connect the above nodes
        node1.next_actions.append(node2)
        node2.next_actions.append(node3)

        # create the pick and place nodes for all parts
        part : KittingPart
        for part in kitting_task.parts:
            target_color = part.part.color
            target_type = part.part.type
            next_action = ActionNode(order_id=order.id,
                           robot=ActionNode.FLOOR_ROBOT,
                           action=ActionNode.ACTION_PICK_PLACE,
                           object_type=ActionNode.OBJECT_TYPE_PART,
                           agv_number=kitting_task.agv_number,
                           tray=target_tray)
            
            part_found = ActionTree.find_part(order_id=order.id,
                                               parts=parts,
                                               target_color=target_color,
                                               target_type=target_type,
                                               target_pose=agv_poses[kitting_task.agv_number-1])
            if part_found is not None:
                part_found.quadrant = part.quadrant
                next_action.add_part(part_found)
            else:
                # insufficient part challenge
                next_action.target_part_color = target_color
                next_action.target_part_type = target_type
                next_action.target_quadrant = part.quadrant
                pass

            node3.next_actions.append(next_action)

        return node1
    
    @staticmethod
    def find_part(order_id : str, parts : List[Part], target_color, target_type, target_pose: Pose):
        """find part that is closest to the target_pose, e.g., pose of an AGV

        Args:
            order_id (str): _description_
            parts (List[Part]): _description_
            target_color (_type_): _description_
            target_type (_type_): _description_
            target_pose (Pose): _description_

        Returns:
            _type_: _description_
        """
        ind = min_ind_pose_dist(poses=[part_.pose for part_ in parts \
                                        if part_.color==target_color and \
                                        part_.type==target_type and \
                                        part_.assigned_order==""],
                                        target_pose=target_pose)
            
        if ind is not None:
            ind = [i for i in range(len(parts)) \
                    if parts[i].color==target_color and\
                        parts[i].type==target_type and\
                        parts[i].assigned_order==""][ind]
            parts[ind].assigned_order = order_id
            return parts[ind]
        else:
            return None

    
    @staticmethod
    def print_tree(root : ActionNode):
        """print some information about the tree

        Args:
            root (ActionNode): _description_
        """
        output = "\n"
        output += "++++++++++++++++++\n"
        output += f"Order id: {root.order_id}\n"
        output += f"Action: {root.action}\n"
        output += f"Object type: {root.object_type}\n"

        for next_action in root.next_actions:
            output += ActionTree.print_tree(next_action)

        return output

    @staticmethod
    def update_cost(node : ActionNode):
        """update the action cost from the node and its child nodes

        Args:
            root (ActionNode): _description_
        """
        if node.is_completed is False:
            node.cost = ActionTree.calculate_cost(node)
        for next_action in node.next_actions:
            ActionTree.update_cost(next_action)

    @staticmethod
    def calculate_cost(node : ActionNode):
        """calculate the cost of performing the current action, e.g.,
        how much the floor robot needs to travel on the rail

        Returns:
            _type_: _description_
        """

        return 1.0
    
    @staticmethod
    def update_cost_queue(node_list : List[ActionNode]):
        """update the action cost from the node and its child nodes

        Args:
            root (ActionNode): _description_
        """
        for node in node_list:
            node.cost = ActionTree.calculate_cost(node)
    
    @staticmethod
    def sort_queue(node_list : List[ActionNode]):
        """sort the action queue according to cost of performing the action

        Args:
            node_list (List[ActionNode]): _description_
        """
        node_list.sort(key=lambda x: x.cost)
    
    @staticmethod
    def insert_remove_part_node(curr_action : ActionNode, parts : List[Part], agv_pose : Pose):
        """insert a remove part action node before the original pick and place node

        Args:
            curr_action (ActionNode): _description_
            parts (List[Part]): _description_
            agv_pose (Pose): _description_

        Raises:
            NotImplementedError: _description_
        """
        # modify the curr_aciton node in place
        # change the action to remove part
        new_action = copy.deepcopy(curr_action)
        new_action.reset_part()
        new_action.next_actions = []
        
        # assign a new part for the new action
        part_found = ActionTree.find_part(order_id=curr_action.order_id,
                                            parts=parts,
                                            target_color=curr_action.part_color,
                                            target_type=curr_action.part_type,
                                            target_pose=agv_pose)
        
        if part_found is None:
            # need to find alternative part or skip this node for now
            report = "❌❌ No matching part found!"
            new_action.target_part_color = curr_action.part_color
            new_action.target_part_type = curr_action.part_type
            new_action.target_quadrant = curr_action.part_quadrant
            # raise NotImplementedError
        else:
            part_found.quadrant = curr_action.part_quadrant
            new_action.add_part(part_found)
            _x, _y, _z = part_found.pose.position.x, part_found.pose.position.y, part_found.pose.position.z
            report = f"✅ Replacement part found at {_x:.1f}, {_y:.1f}, {_z:.1f}!"
        
        new_action.reset_service_calls(n_service_call=3)
        
        curr_action.action = ActionNode.ACTION_REMOVE_PART_FROM_AGV
        curr_action.next_actions = [new_action]

        # reset service calls for both action nodes
        curr_action.reset_service_calls(n_service_call=1)

        return report