import rclpy
from rclpy.node import Node
from custom_msgs.srv import PickupTray


class PickupTrayServer(Node):

    def __init__(self):
        super().__init__('pickup_tray_server')

        self.service = self.create_service(PickupTray, '/ariac/custom_pickup_tray', self.pickup_tray)

    # def pickup_tray(self, robot, tray_id, tray_pose, tray_station):

    def pickup_tray(self, request, response):
        """Emulate picking up a tray.

        Args:
            request: The request message containing tray information.
            response: The response message indicating success.

        Returns:
            PickupTray.Response: The response message indicating success.
        """
        # Get tray ID
        tray_id = request.tray_id

        # Get tray pose components
        tray_pose_x = request.tray_pose.position.x
        tray_pose_y = request.tray_pose.position.y
        tray_pose_z = request.tray_pose.position.z
        tray_orientation_x = request.tray_pose.orientation.x
        tray_orientation_y = request.tray_pose.orientation.y
        tray_orientation_z = request.tray_pose.orientation.z
        tray_orientation_w = request.tray_pose.orientation.w

        # Print the pickup tray message
        self.get_logger().info(f"Picking up tray {tray_id} located at "
                               f"[{tray_pose_x:.2f} {tray_pose_y:.2f} {tray_pose_z:.2f}] "
                               f"[{tray_orientation_x:.2f} {tray_orientation_y:.2f} {tray_orientation_z:.2f} {tray_orientation_w:.2f}]")
        
        # Set the response success to True
        response.success = True

        return response

