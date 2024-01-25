import math
from typing import List, Tuple
import PyKDL
from dataclasses import dataclass
from geometry_msgs.msg import (
    Pose,
    PoseStamped, 
    Vector3,
    Quaternion
)

from ariac_msgs.msg import (
    PartPose as PartPoseMsg,
    KitTrayPose as KitTrayPoseMsg,
    Part as PartMsg,
    Order as OrderMsg
)

from ariac_msgs.msg import Part as PartMsg

import numpy as np
from typing import List

def multiply_pose(p1: Pose, p2: Pose) -> Pose:
    '''
    Use KDL to multiply two poses together.
    Args:
        p1 (Pose): Pose of the first frame
        p2 (Pose): Pose of the second frame
    Returns:
        Pose: Pose of the resulting frame
    '''

    o1 = p1.orientation
    frame1 = PyKDL.Frame(
        PyKDL.Rotation.Quaternion(o1.x, o1.y, o1.z, o1.w),
        PyKDL.Vector(p1.position.x, p1.position.y, p1.position.z))

    o2 = p2.orientation
    frame2 = PyKDL.Frame(
        PyKDL.Rotation.Quaternion(o2.x, o2.y, o2.z, o2.w),
        PyKDL.Vector(p2.position.x, p2.position.y, p2.position.z))

    frame3 = frame1 * frame2

    # return the resulting pose from frame3
    pose = Pose()
    pose.position.x = frame3.p.x()
    pose.position.y = frame3.p.y()
    pose.position.z = frame3.p.z()

    q = frame3.M.GetQuaternion()
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]

    return pose


def rpy_from_quaternion(q: Quaternion) -> Tuple[float, float, float]:
    ''' 
    Use KDL to convert a quaternion to euler angles roll, pitch, yaw.
    Args:
        q (Quaternion): quaternion to convert
    Returns:
        Tuple[float, float, float]: roll, pitch, yaw
    '''
    
    R = PyKDL.Rotation.Quaternion(q.x, q.y, q.z, q.w)
    return R.GetRPY()


def rad_to_deg_str(radians: float) -> str:
    '''
    Converts radians to degrees in the domain [-PI, PI]
    Args:
        radians (float): value in radians
    Returns:
        str: String representing the value in degrees
    '''
    
    degrees = math.degrees(radians)
    if degrees > 180:
        degrees = degrees - 360
    elif degrees < -180:
        degrees = degrees + 360

    if -1 < degrees < 1:
        degrees = 0 
    
    return f'{degrees:.0f}' + chr(176)

@dataclass
class AdvancedLogicalCameraImage:
    '''
    Class to store information about a AdvancedLogicalCameraImageMsg
    '''
    _part_poses: PartPoseMsg
    _tray_poses: KitTrayPoseMsg
    _sensor_pose: Pose
    

@dataclass
class KittingPart:
    '''
    Class to store information about a KittingPartMsg.
    '''
    _quadrant: int
    _part: PartMsg

    @property
    def quadrant(self) -> int:
        '''
        Returns the quadrant of the part.

        Returns:
            int: The quadrant of the part.
        '''
        return self._quadrant

    @property
    def part(self) -> PartMsg:
        '''
        Returns the type of the part.

        Returns:
            PartMsg: The type of the part.
        '''
        return self._part


@dataclass
class KittingTask:
    '''
    Class to store information about a KittingTaskMsg.
    '''
    _agv_number: int
    _tray_id: int
    _destination: int
    _parts:  List[KittingPart]

    @property
    def agv_number(self) -> int:
        '''
        Returns the AGV number.

        Returns:
            int: The AGV number.
        '''
        return self._agv_number

    @property
    def tray_id(self) -> int:
        '''
        Returns the tray ID.

        Returns:
            int: The tray ID.
        '''
        return self._tray_id

    @property
    def destination(self) -> int:
        '''
        Returns the destination.

        Returns:
            int: The destination.
        '''
        return self._destination

    @property
    def parts(self) -> List[KittingPart]:
        '''
        Returns the list of parts.

        Returns:
            List[KittingPart]: The list of parts.
        '''
        return self._parts


@dataclass
class AssemblyPart:
    '''
    Class to store information about a AssemblyPartMsg.
    '''

    _part: PartMsg
    _assembled_pose: PoseStamped
    _install_direction: Vector3

    @property
    def part(self) -> PartMsg:
        '''
        Returns the type of the part.

        Returns:
            PartMsg: The type of the part.
        '''
        return self._part

    @property
    def assembled_pose(self) -> PoseStamped:
        '''
        Returns the assembled pose of the part.

        Returns:
            PoseStamped: The assembled pose of the part.
        '''
        return self._assembled_pose

    @property
    def install_direction(self) -> Vector3:
        '''
        Returns the install direction of the part.

        Returns:
            Vector3: The install direction of the part.
        '''
        return self._install_direction


@dataclass
class AssemblyTask:
    '''
    Class to store information about a AssemblyTaskMsg.
    '''

    _agv_numbers: List[int]
    _station: int
    _parts:  List[AssemblyPart]

    @property
    def agv_numbers(self) -> List[int]:
        '''
        Returns the list of AGV numbers.

        Returns:
            List[int]: The list of AGV numbers.
        '''
        return self._agv_numbers

    @property
    def station(self) -> int:
        '''
        Returns the station.

        Returns:
            int: The station.
        '''
        return self._station

    @property
    def parts(self) -> List[AssemblyPart]:
        '''
        Returns the list of parts.

        Returns:
            List[AssemblyPart]: The list of parts.
        '''
        return self._parts


@dataclass
class CombinedTask:
    '''
    Class to store information about a CombinedTaskMsg.
    '''

    _station: int
    _parts:  List[AssemblyPart]

    @property
    def station(self) -> int:
        '''
        Returns the station.

        Returns:
            int: The station.
        '''
        return self._station

    @property
    def parts(self) -> List[AssemblyPart]:
        '''
        Returns the list of parts.

        Returns:
            List[AssemblyPart]: The list of parts.
        '''
        return self._parts


class Order:
    ''' 
    Class to store one order message from the topic /ariac/orders.
    '''

    def __init__(self, msg: OrderMsg) -> None:
        self.order_id = msg.id
        self.order_type = msg.type
        self.order_priority = msg.priority

        if self.order_type == OrderMsg.KITTING:
            self.order_task = KittingTask(msg.kitting_task.agv_number,
                                          msg.kitting_task.tray_id,
                                          msg.kitting_task.destination,
                                          msg.kitting_task.parts)

        elif self.order_type == OrderMsg.ASSEMBLY:
            self.order_task = AssemblyTask(msg.assembly_task.agv_numbers,
                                           msg.assembly_task.station,
                                           msg.assembly_task.parts)
        elif self.order_type == OrderMsg.COMBINED:
            self.order_task = CombinedTask(msg.combined_task.station, msg.combined_task.parts)
        else:
            self.order_task = None

def dist_two_poses(pose1: Pose, pose2: Pose):
    """return the eucledian distance between two PartPoseMsg

    Args:
        pose1 (PartPoseMsg): _description_
        pose2 (PartPoseMsg): _description_

    Returns:
        _type_: _description_
    """
    positions = [np.array([P.position.x,P.position.y,P.position.z])
                 for P in [pose1, pose2]]
    return np.linalg.norm(positions[0] - positions[1])

def min_ind_pose_dist(poses : List[Pose], target_pose : Pose):
    if len(poses) == 0:
        return None

    positions = np.array([[P.position.x,P.position.y,P.position.z]
                 for P in poses])
    target_position = np.array([[target_pose.position.x,target_pose.position.y,target_pose.position.z]])
    d = np.linalg.norm(positions-target_position, axis=1)
    return np.argmin(d)

def min_sum_pose_dist(poses : List[Pose], target_poses : List[Pose]):
    target_positions = np.array([[tp.position.x,tp.position.y,tp.position.z] for tp in target_poses])
    d = []
    for pose in poses:
        pos = np.array([[pose.position.x,pose.position.y,pose.position.z]])
        d.append(np.sum(np.linalg.norm(pos-target_positions,axis=1)))
    d = np.array(d)
    return np.argmin(d)

def print_assigned_order(list_):
    """print the assigned order for Tray or Part object

    Args:
        list_ (_type_): list of Tray or Part
    """
    for item in list_:
        print(item.assigned_order)

def print_poses(list_):
    output = ""

    for item in list_:
        output += (str(item.pose)+"\n")
    return output

class Tray:
    def __init__(self, tray_msg: KitTrayPoseMsg, assigned_order: str = ""):
        self.pose = tray_msg.pose
        self.id = tray_msg.id
        self.assigned_order = assigned_order

class Part:

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

    def __init__(self, part_msg: PartPoseMsg, assigned_order : str = "", part_id : int=0, quadrant = 0):
        self.pose = part_msg.pose
        self.color = part_msg.part.color
        self.type = part_msg.part.type
        self.assigned_order = assigned_order
        self.quadrant = quadrant
        self.id = part_id

    def print(self):
        return f"{Part._part_colors[self.color].capitalize()} {Part._part_types[self.type].capitalize()} {self.assigned_order} "

def gen_pose(x,y,z,roll,pitch,yaw):
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z

    (x_,y_,z_,w_) = PyKDL.Rotation.RPY(roll,pitch,yaw).GetQuaternion()
    p.orientation.x = x_
    p.orientation.y = y_
    p.orientation.z = z_
    p.orientation.w = w_

    return p