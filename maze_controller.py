#!/usr/bin/env python3
"""
Main Maze Controller
Orchestrates all escape maze components
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class MazeController(Node):
    def __init__(self):
        super().__init__('maze_controller')
        
        # Subscribers to all system components
        self.create_subscription(String, '/maze/clue_detected', self.clue_handler, 10)
        self.create_subscription(String, '/maze/action', self.action_handler, 10)
        self.create_subscription(String, '/maze/status', self.status_handler, 10)
        self.create_subscription(String, '/navigation/status', self.nav_handler, 10)
        self.create_subscription(String, '/system/alerts', self.alert_handler, 10)
        self.create_subscription(String, '/emergency/stop', self.emergency_handler, 10)
        
        # Publisher for overall mission control
        self.mission_pub = self.create_publisher(String, '/mission/control', 10)
        self.ui_pub = self.create_publisher(String, '/ui/messages', 10)
        
        # Mission state
        self.mission_state = 'READY'
        self.current_zone = 'ENTRY'
        self.mission_log = []
        
        # Zone sequence from PDF
        self.zone_sequence = ['ENTRY_GATE', 'COLOR_HALL', 'QR_CHAMBER', 'ZONE_C', 'FINAL_EXIT']
        self.current_zone_index = 0
        
        # Mission parameters
        self.max_clues = 10
        self.time_limit = 600  # 10 minutes in seconds
        self.start_time = None
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("ESCAPE MAZE CONTROLLER INITIALIZED")
        self.get_logger().info("=" * 50)
        
        # Start mission
        self.create_timer(2.0, self.start_mission)

    def start_mission(self):
        """Start the escape maze mission"""
        self.mission_state = 'ACTIVE'
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        
        start_msg = String()
        start_msg.data = json.dumps({
            'command': 'START_MISSION',
            'state': self.mission_state,
            'zone': self.zone_sequence[0],
            'message': 'Escape Maze Mission Started!'
        })
        
        self.mission_pub.publish(start_msg)
        self.ui_pub.publish(String(data="MISSION STARTED! Find the escape path!"))
        
        self.log_event("Mission started")
        self.get_logger().info("Mission started - Good luck!")

    def clue_handler(self, msg):
        """Handle detected clues"""
        if self.mission_state != 'ACTIVE':
            return
        
        try:
            clue_data = json.loads(msg.data)
            clue_type = clue_data.get('type')
            clue_value = clue_data.get('value')
            
            self.log_event(f"Clue detected: {clue_type}={clue_value}")
            
            # Update UI
            ui_msg = f"Clue: {clue_value}"
            self.ui_pub.publish(String(data=ui_msg))
            
            # Check for key acquisition
            if clue_type == 'shape' and clue_value == 'star':
                self.handle_key_acquisition()
                
        except json.JSONDecodeError:
            self.log_event(f"Raw clue: {msg.data}")

    def action_handler(self, msg):
        """Handle puzzle actions"""
        action = msg.data
        self.log_event(f"Action: {action}")
        
        if action == 'key_acquired':
            self.handle_key_acquisition()
        elif action == 'victory':
            self.complete_mission()

    def status_handler(self, msg):
        """Handle status updates"""
        self.ui_pub.publish(msg)  # Forward to UI

    def nav_handler(self, msg):
        """Handle navigation status"""
        status = msg.data
        
        if 'succeeded' in status.lower():
            # Zone completed, move to next
            self.current_zone_index = min(self.current_zone_index + 1, 
                                        len(self.zone_sequence) - 1)
            self.current_zone = self.zone_sequence[self.current_zone_index]
            
            self.log_event(f"Entered zone: {self.current_zone}")
            
            # Check if mission complete
            if self.current_zone == 'FINAL_EXIT':
                self.check_mission_completion()

    def alert_handler(self, msg):
        """Handle system alerts"""
        self.log_event(f"ALERT: {msg.data}")
        self.ui_pub.publish(String(data=f"?? {msg.data}"))
        
        # Critical alerts may pause mission
        if 'CRITICAL' in msg.data:
            self.pause_mission()

    def emergency_handler(self, msg):
        """Handle emergency stops"""
        self.log_event(f"EMERGENCY: {msg.data}")
        self.mission_state = 'PAUSED'
        
        emergency_msg = String()
        emergency_msg.data = json.dumps({
            'command': 'EMERGENCY_STOP',
            'state': self.mission_state,
            'message': 'Emergency stop activated'
        })
        
        self.mission_pub.publish(emergency_msg)
        self.ui_pub.publish(String(data="?? EMERGENCY STOP ACTIVATED"))

    def handle_key_acquisition(self):
        """Handle key symbol acquisition"""
        self.log_event("KEY SYMBOL ACQUIRED!")
        self.ui_pub.publish(String(data="?? KEY ACQUIRED! Head to exit!"))
        
        # Unlock final exit
        key_msg = String()
        key_msg.data = json.dumps({
            'command': 'UNLOCK_EXIT',
            'state': self.mission_state,
            'message': 'Exit unlocked with key'
        })
        
        self.mission_pub.publish(key_msg)

    def check_mission_completion(self):
        """Check if mission is complete"""
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        elapsed = current_time - self.start_time
        
        # Check conditions for completion
        if elapsed > self.time_limit:
            self.mission_state = 'TIMEOUT'
            self.end_mission("Time's up! Mission failed.")
        else:
            self.complete_mission()

    def complete_mission(self):
        """Complete the mission successfully"""
        self.mission_state = 'COMPLETED'
        
        victory_msg = String()
        victory_msg.data = json.dumps({
            'command': 'MISSION_COMPLETE',
            'state': self.mission_state,
            'time_elapsed': self.get_clock().now().seconds_nanoseconds()[0] - self.start_time,
            'message': 'Escape successful!'
        })
        
        self.mission_pub.publish(victory_msg)
        self.ui_pub.publish(String(data="?? ESCAPE SUCCESSFUL! MAZE COMPLETED!"))
        
        self.log_event("Mission completed successfully")
        self.get_logger().info("=" * 50)
        self.get_logger().info("MISSION COMPLETE - ESCAPE SUCCESSFUL!")
        self.get_logger().info("=" * 50)
        
        # Play victory sequence
        self.create_timer(1.0, self.victory_sequence)

    def pause_mission(self):
        """Pause the mission"""
        self.mission_state = 'PAUSED'
        pause_msg = String()
        pause_msg.data = json.dumps({
            'command': 'PAUSE_MISSION',
            'state': self.mission_state
        })
        self.mission_pub.publish(pause_msg)

    def resume_mission(self):
        """Resume paused mission"""
        self.mission_state = 'ACTIVE'
        resume_msg = String()
        resume_msg.data = json.dumps({
            'command': 'RESUME_MISSION',
            'state': self.mission_state
        })
        self.mission_pub.publish(resume_msg)

    def end_mission(self, reason):
        """End mission (failed or aborted)"""
        self.mission_state = 'ENDED'
        
        end_msg = String()
        end_msg.data = json.dumps({
            'command': 'END_MISSION',
            'state': self.mission_state,
            'reason': reason
        })
        
        self.mission_pub.publish(end_msg)
        self.ui_pub.publish(String(data=f"Mission ended: {reason}"))

    def victory_sequence(self):
        """Play victory sequence"""
        messages = [
            "Congratulations!",
            "You escaped the maze!",
            "All puzzles solved!",
            "Mission accomplished!"
        ]
        
        for i, msg in enumerate(messages):
            self.create_timer(i * 1.5, lambda m=msg: self.ui_pub.publish(String(data=m)))

    def log_event(self, event):
        """Log mission events"""
        timestamp = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time
        log_entry = f"[{timestamp}s] {event}"
        self.mission_log.append(log_entry)
        self.get_logger().info(log_entry)

def main(args=None):
    rclpy.init(args=args)
    node = MazeController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
