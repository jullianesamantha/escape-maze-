#!/usr/bin/env python3
"""
Puzzle Logic Engine for Escape Maze
Interprets clues and makes navigation decisions
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
import json
import re

class PuzzleLogicEngine(Node):
    def __init__(self):
        super().__init__('puzzle_logic_engine')
        
        # Subscribers
        self.create_subscription(String, '/maze/clue_detected', self.clue_callback, 10)
        self.create_subscription(String, '/navigation/status', self.nav_callback, 10)
        
        # Publishers
        self.nav_goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.action_pub = self.create_publisher(String, '/maze/action', 10)
        self.status_pub = self.create_publisher(String, '/maze/status', 10)
        
        # Puzzle state
        self.current_zone = 'entry_gate'
        self.has_key = False
        self.solved_puzzles = set()
        self.last_clue = None
        
        # Puzzle rules from PDF
        self.puzzle_rules = {
            'shape': {
                'triangle': {'action': 'turn_right', 'next_zone': 'color_hall'},
                'square': {'action': 'turn_left', 'next_zone': 'color_hall'},
                'circle': {'action': 'move_forward', 'next_zone': 'color_hall'},
                'star': {'action': 'acquire_key', 'next_zone': 'exit'}
            },
            'color': {
                'red': {'action': 'stop_reroute', 'behavior': 'reverse_turn'},
                'green': {'action': 'move_forward', 'speed': 'normal'},
                'yellow': {'action': 'slow_down', 'speed': 'slow'},
                'blue': {'action': 'turn_left', 'angle': 90}
            },
            'qr': {
                'pattern': {
                    r'proceed to zone (\w+)': 'goto_zone',
                    r'go to zone (\w+)': 'goto_zone',
                    r'backward (\d+(?:\.\d+)?)m then turn left': 'reverse_turn_left',
                    r'seek the key rune': 'hint_key',
                    r'key is found in zone[ _]?c': 'goto_zone_c'
                }
            }
        }
        
        # Waypoints for each zone
        self.zone_waypoints = {
            'entry_gate': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            'zone_1': {'x': 1.0, 'y': 0.0, 'yaw': 0.0},
            'color_hall': {'x': 2.0, 'y': 1.0, 'yaw': 1.57},
            'qr_chamber': {'x': 1.5, 'y': 2.0, 'yaw': 3.14},
            'zone_c': {'x': 2.5, 'y': 1.5, 'yaw': 0.0},
            'final_exit': {'x': 3.0, 'y': 0.0, 'yaw': 0.0}
        }
        
        self.get_logger().info("Puzzle Logic Engine initialized")
        self.publish_status("Initialized - Starting at Entry Gate")

    def clue_callback(self, msg):
        """Process detected clues"""
        try:
            clue_data = json.loads(msg.data)
            clue_type = clue_data.get('type')
            clue_value = clue_data.get('value')
            clue_zone = clue_data.get('zone', 'unknown')
            
            if not clue_type or not clue_value:
                return
            
            # Avoid processing same clue multiple times
            clue_id = f"{clue_type}:{clue_value}:{clue_zone}"
            if clue_id == self.last_clue:
                return
            self.last_clue = clue_id
            
            self.get_logger().info(f"Processing clue: {clue_type}={clue_value} in {clue_zone}")
            self.publish_status(f"Processing: {clue_value}")
            
            # Execute puzzle logic based on clue type
            if clue_type == 'shape':
                self.handle_shape_clue(clue_value, clue_zone)
            elif clue_type == 'color':
                self.handle_color_clue(clue_value, clue_zone)
            elif clue_type == 'qr':
                self.handle_qr_clue(clue_value, clue_zone)
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid clue JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"Clue processing error: {e}")

    def handle_shape_clue(self, shape, zone):
        """Handle shape-based puzzle logic"""
        if shape in self.puzzle_rules['shape']:
            rule = self.puzzle_rules['shape'][shape]
            
            if shape == 'star':
                # Found the key symbol!
                self.has_key = True
                self.solved_puzzles.add('key_acquired')
                self.publish_status("KEY ACQUIRED! Proceed to exit.")
                self.send_action("key_acquired")
                
                # Navigate to exit
                self.set_navigation_goal('final_exit')
            else:
                # Regular shape clue
                action = rule['action']
                self.send_action(action)
                
                if action == 'turn_right':
                    self.execute_turn(1.0, 2.0)  # Turn right for 2 seconds
                elif action == 'turn_left':
                    self.execute_turn(-1.0, 2.0)  # Turn left for 2 seconds
                elif action == 'move_forward':
                    self.execute_move(0.3, 3.0)  # Move forward
                
                # Move to next zone
                if 'next_zone' in rule:
                    self.set_navigation_goal(rule['next_zone'])
                    self.current_zone = rule['next_zone']

    def handle_color_clue(self, color, zone):
        """Handle color-based puzzle logic"""
        if color in self.puzzle_rules['color']:
            rule = self.puzzle_rules['color'][color]
            action = rule['action']
            
            self.send_action(f"color_{action}")
            
            if color == 'red':
                # Stop and reroute
                self.publish_status("RED DETECTED: Stopping and rerouting")
                self.execute_move(-0.2, 1.0)  # Reverse
                self.execute_turn(0.5, 2.0)   # Turn
                self.execute_move(0.3, 2.0)   # Move forward
            elif color == 'green':
                self.execute_move(0.3, 3.0)
            elif color == 'yellow':
                self.execute_move(0.15, 2.0)  # Slow movement
            elif color == 'blue':
                self.execute_turn(-0.5, 1.5)  # Turn left

    def handle_qr_clue(self, qr_text, zone):
        """Handle QR code instructions"""
        self.get_logger().info(f"QR Instruction: {qr_text}")
        self.publish_status(f"QR: {qr_text}")
        
        # Parse QR instructions using regex patterns
        for pattern, action_type in self.puzzle_rules['qr']['pattern'].items():
            match = re.search(pattern, qr_text.lower())
            if match:
                if action_type == 'goto_zone':
                    zone_name = match.group(1).upper()
                    self.set_navigation_goal(f'zone_{zone_name}')
                elif action_type == 'goto_zone_c':
                    self.set_navigation_goal('zone_c')
                elif action_type == 'hint_key':
                    self.publish_status("Hint: Look for star symbol in Zone C")
                elif action_type == 'reverse_turn_left':
                    distance = float(match.group(1))
                    self.execute_move(-0.2, distance/0.2)  # Reverse for specified distance
                    self.execute_turn(-0.5, 2.0)           # Turn left
                break

    def execute_move(self, speed, duration):
        """Execute linear movement"""
        twist = Twist()
        twist.linear.x = speed
        self.cmd_vel_pub.publish(twist)
        
        # Stop after duration (simplified - in real implementation use timers)
        self.create_timer(duration, lambda: self.stop_movement())

    def execute_turn(self, angular_speed, duration):
        """Execute angular movement"""
        twist = Twist()
        twist.angular.z = angular_speed
        self.cmd_vel_pub.publish(twist)
        
        self.create_timer(duration, lambda: self.stop_movement())

    def stop_movement(self):
        """Stop all movement"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def set_navigation_goal(self, zone_name):
        """Set navigation goal for the specified zone"""
        if zone_name in self.zone_waypoints:
            wp = self.zone_waypoints[zone_name]
            
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = wp['x']
            goal.pose.position.y = wp['y']
            goal.pose.position.z = 0.0
            
            # Convert yaw to quaternion
            import math
            goal.pose.orientation.z = math.sin(wp['yaw'] / 2.0)
            goal.pose.orientation.w = math.cos(wp['yaw'] / 2.0)
            
            self.nav_goal_pub.publish(goal)
            self.publish_status(f"Navigating to {zone_name}")
            self.current_zone = zone_name

    def send_action(self, action):
        """Publish action message"""
        action_msg = String()
        action_msg.data = action
        self.action_pub.publish(action_msg)

    def publish_status(self, status):
        """Publish status update"""
        status_msg = String()
        status_msg.data = f"[{self.current_zone}] {status}"
        self.status_pub.publish(status_msg)
        self.get_logger().info(status)

    def nav_callback(self, msg):
        """Handle navigation status updates"""
        status = msg.data
        if 'succeeded' in status.lower() and self.current_zone == 'final_exit' and self.has_key:
            self.publish_status("ESCAPE SUCCESSFUL! Maze completed!")
            self.send_action("victory")
            # Play victory sound or LED effect

def main(args=None):
    rclpy.init(args=args)
    node = PuzzleLogicEngine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
