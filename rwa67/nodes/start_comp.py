#!/usr/bin/env python3
'''
'''

import rclpy
from rwa67.competition_interface import CompetitionInterface
from ariac_msgs.msg import CompetitionState
# from rwa67.custom_servers import ServiceServerNode



def main(args=None):
    rclpy.init(args=args)
    interface = CompetitionInterface()
    interface.start_competition()
    interface.parse_incoming_order = True

    while rclpy.ok():
        try:
            rclpy.spin_once(interface)
            if interface.competition_state == CompetitionState.ENDED:
                break
        except KeyboardInterrupt:
            break

    interface.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()