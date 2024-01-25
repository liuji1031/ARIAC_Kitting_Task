#!/usr/bin/env python3
'''
'''

import rclpy
from rwa67.pickup_part_server import PickupPartServer


def main(args=None):
    rclpy.init(args=args)
    server_node = PickupPartServer()
    rclpy.spin(server_node)
    server_node.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()