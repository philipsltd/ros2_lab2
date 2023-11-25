#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import action_msgs.msg
from std_srvs.srv import Trigger

class NavigationNode(Node):
    
    def __init__(self):
        super().__init__('navigation_node')
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        self.start_navigation_service = self.create_service(Trigger, '/start_navigation', self.send_goal_callback)
        self.get_logger().info('Service started')

    def send_goal_callback(self, request, response):

        # Create a NavigateToPose goal
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = -8.037 # Replace with your desired coordinates
        goal_pose.pose.position.y = 4.484  # Replace with your desired coordinates
        goal_pose.pose.orientation.w = -3.116  # Replace with your desired coordinates
        
        # Send the goal
        self.goal_publisher.publish(goal_pose)
        self.get_logger().info('Sent Goal!')

        response.success = True
        response.message = 'Sent Goal Successfully!'

        return response

def main(args=None):
    rclpy.init(args=args)

    node = NavigationNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()