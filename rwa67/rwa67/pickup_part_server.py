import rclpy
from rclpy.node import Node
from ariac_msgs.msg import Part
from custom_msgs.srv import PickupPart

class PickupPartServer(Node):

    def __init__(self):
        super().__init__('pickup_part_server')

        self.service = self.create_service(PickupPart, '/ariac/custom_pickup_part', self.pickup_part)


    def pickup_part(self, request, response):
        """Emulate picking up a part from a bin.

        Args:
            request: The request message containing part information.
            response: The response message indicating success.

        Returns:
            PartPickup.Response: The response message indicating success.
        """
        # Map robot ID to robot name
        robot_names = ["FLOOR_ROBOT", "CEILING_ROBOT"]
        robot_name = robot_names[request.robot]

        # Map part type ID to part type name
        part_types = {Part.PUMP: "Pump",
                      Part.BATTERY: "Battery",
                      Part.REGULATOR: "Regulator",
                      Part.SENSOR: "Sensor"}
        part_type_name = part_types[request.part_type]

        # Map part color ID to part color name
        part_colors = {
            Part.RED: 'Red',
            Part.BLUE: 'Blue',
            Part.GREEN: 'Green',
            Part.ORANGE: 'Orange',
            Part.PURPLE: 'Purple',
        }
        part_color_name = part_colors[request.part_color]

        # Get part pose components
        part_pose_x = request.part_pose.position.x
        part_pose_y = request.part_pose.position.y
        part_pose_z = request.part_pose.position.z
        part_orientation_x = request.part_pose.orientation.x
        part_orientation_y = request.part_pose.orientation.y
        part_orientation_z = request.part_pose.orientation.z
        part_orientation_w = request.part_pose.orientation.w

        # Print the pickup message
        self.get_logger().info(f"Picking up {part_color_name} {part_type_name} located at "
                               f"[{part_pose_x:.2f} {part_pose_y:.2f} {part_pose_z:.2f}] "
                               f"[{part_orientation_x:.2f} {part_orientation_y:.2f} {part_orientation_z:.2f} {part_orientation_w:.2f}]")
        
        # Set the response success to True
        response.success = True

        return response