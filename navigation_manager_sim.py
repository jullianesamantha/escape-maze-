#!/usr/bin/env python3
"""
Maze Navigation Manager
Nav2 integration with puzzle-based waypoint management
"""

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import json
import math

class MazeNavigationManager(Node):
    def __init__(self):
        super().__init__('maze_navigation_manager')
        
        # Initialize Nav2 navigator
        self.navigator = BasicNavigator()
        
        # Subscribers
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(String, '/maze/action', self.action_callback, 10)
        
        # Publishers
        self.nav_status_pub = self.create_publisher(String, '/navigation/status', 10)
        self.feedback_pub = self.create_publisher(String, '/navigation/feedback', 10)
        
        # Wait for Nav2 to be active
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 active - Maze Navigation Manager ready")
        
        # Current navigation state
        self.navigating = False
        self.current_goal = None
        
        # Timer for navigation monitoring
        self.nav_timer = self.create_timer(0.5, self.navigation_monitor)

    def goal_callback(self, msg):
        """Handle new navigation goal"""
        self.current_goal = msg
        self.navigating = True
        
        goal_str = f"x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}"
        self.get_logger().info(f"Received navigation goal: {goal_str}")
        self.publish_status(f"Navigating to: {goal_str}")
        
        # Send goal to Nav2
        self.navigator.goToPose(msg)

    def action_callback(self, msg):
        """Handle puzzle actions that affect navigation"""
        action = msg.data
        
        if action == 'emergency_stop':
            self.emergency_stop()
        elif action == 'pause_navigation':
            self.pause_navigation()
        elif action == 'resume_navigation':
            self.resume_navigation()

    def emergency_stop(self):
        """Emergency stop - cancel all navigation"""
        if self.navigating:
            self.navigator.cancelTask()
            self.navigating = False
            self.publish_status("EMERGENCY STOP - Navigation cancelled")

    def pause_navigation(self):
        """Pause current navigation"""
        if self.navigating:
            # In Nav2, we'd typically cancel and resume later
            # For simulation, we'll just slow down
            self.publish_status("Navigation paused")
            # Could implement velocity reduction here

    def resume_navigation(self):
        """Resume navigation"""
        if self.current_goal and not self.navigating:
            self.navigator.goToPose(self.current_goal)
            self.navigating = True
            self.publish_status("Navigation resumed")

    def navigation_monitor(self):
        """Monitor navigation progress"""
        if not self.navigating:
            return
        
        # Check if task is complete
        if self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            
            if result == TaskResult.SUCCEEDED:
                self.publish_status("Navigation succeeded - Goal reached")
                self.navigating = False
            elif result == TaskResult.CANCELED:
                self.publish_status("Navigation canceled")
                self.navigating = False
            elif result == TaskResult.FAILED:
                self.publish_status("Navigation failed - Retrying...")
                # Auto-retry
                if self.current_goal:
                    self.navigator.goToPose(self.current_goal)
            return
        
        # Publish navigation feedback
        feedback = self.navigator.getFeedback()
        if feedback:
            feedback_data = {
                'distance_remaining': feedback.distance_remaining if hasattr(feedback, 'distance_remaining') else 0.0,
                'navigating': True,
                'current_pose': {
                    'x': feedback.current_pose.pose.position.x if hasattr(feedback, 'current_pose') else 0.0,
                    'y': feedback.current_pose.pose.position.y if hasattr(feedback, 'current_pose') else 0.0
                }
            }
            
            feedback_msg = String()
            feedback_msg.data = json.dumps(feedback_data)
            self.feedback_pub.publish(feedback_msg)

    def publish_status(self, status):
        """Publish navigation status"""
        status_msg = String()
        status_msg.data = status
        self.nav_status_pub.publish(status_msg)
        self.get_logger().info(f"Navigation: {status}")

    def create_zone_pose(self, x, y, yaw):
        """Helper to create pose for zone waypoints"""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return pose

def main(args=None):
    rclpy.init(args=args)
    node = MazeNavigationManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
