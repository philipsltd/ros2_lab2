#!/usr/bin/env python3

import rclpy
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import action_msgs.msg

def send_goal():
    rclpy.init()

    node = rclpy.create_node('send_goals_node')

    # Create an action client for NavigateToPose
    action_client = node.create_client(NavigateToPose, 'navigate_to_pose')

    # Wait for the action server to start
    if not action_client.wait_for_server(timeout_sec=20.0):
        node.get_logger().error('Action server not available')
        return

    # Create a NavigateToPose goal
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose = PoseStamped()
    goal_msg.pose.header.frame_id = 'map'
    goal_msg.pose.pose.position.x = 1.0  # Replace with your desired coordinates
    goal_msg.pose.pose.position.y = 2.0
    goal_msg.pose.pose.orientation.w = 1.0

    # Send the goal
    future_goal_handle = action_client.send_goal_async(goal_msg)

    rclpy.spin_until_future_complete(node, future_goal_handle)

    # Handle the result
    goal_handle = future_goal_handle.result()
    if not goal_handle.accepted:
        node.get_logger().error('Goal rejected')
        return

    result_future = action_client.get_result_async(goal_handle.goal_id)
    rclpy.spin_until_future_complete(node, result_future)
    result = result_future.result()

    if result.code != action_msgs.msg.GoalStatus.STATUS_SUCCEEDED:
        node.get_logger().error(f'Goal failed with status code {result.code}')
        return

    node.get_logger().info('Goal succeeded!')

    rclpy.shutdown()

if __name__ == '__main__':
    send_goal()
