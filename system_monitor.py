#!/usr/bin/env python3
"""
System Monitor for Escape Maze Robot
Combines diagnostics monitoring and mock publishing
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from datetime import datetime
from pathlib import Path
import json
import random

class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor')
        
        # Publisher for simulated diagnostics
        self.diag_pub = self.create_publisher(BatteryState, '/robot/diagnostics', 10)
        
        # Publisher for system alerts
        self.alert_pub = self.create_publisher(String, '/system/alerts', 10)
        
        # Publisher for maze-specific status
        self.maze_status_pub = self.create_publisher(String, '/maze/system_status', 10)
        
        # Timer for publishing diagnostics
        self.diag_timer = self.create_timer(2.0, self.publish_diagnostics)
        
        # Timer for system status updates
        self.status_timer = self.create_timer(5.0, self.publish_system_status)
        
        # Log directory
        log_dir = Path(__file__).parent / 'logs'
        log_dir.mkdir(parents=True, exist_ok=True)
        self.log_file = log_dir / 'system_monitor.log'
        
        # System state
        self.battery_level = 85.0  # Start at 85%
        self.temperature = 36.0    # Start at 36°C
        self.cpu_usage = 15.0      # Simulated CPU usage
        self.memory_usage = 45.0   # Simulated memory usage
        
        # Thresholds
        self.LOW_BATTERY = 20.0
        self.CRITICAL_BATTERY = 10.0
        self.HIGH_TEMP = 60.0
        self.CRITICAL_TEMP = 70.0
        
        # Maze performance metrics
        self.clues_found = 0
        self.zones_visited = set()
        self.puzzles_solved = 0
        
        self.get_logger().info("System Monitor initialized")

    def publish_diagnostics(self):
        """Publish simulated diagnostics data"""
        # Simulate battery drain
        drain = random.uniform(0.1, 0.3)
        self.battery_level = max(0.0, self.battery_level - drain)
        
        # Simulate temperature changes
        temp_change = random.uniform(-0.5, 1.0)
        self.temperature = max(25.0, min(75.0, self.temperature + temp_change))
        
        # Create BatteryState message
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.voltage = 12.0 * (self.battery_level / 100.0)
        msg.temperature = self.temperature
        msg.percentage = self.battery_level / 100.0
        msg.present = True
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        
        # Add maze-specific diagnostics
        msg.serial_number = f"CLUES:{self.clues_found}:ZONES:{len(self.zones_visited)}"
        
        self.diag_pub.publish(msg)
        
        # Log and check thresholds
        self.check_thresholds()

    def check_thresholds(self):
        """Check system thresholds and publish alerts if needed"""
        alerts = []
        
        # Battery alerts
        if self.battery_level <= self.CRITICAL_BATTERY:
            alerts.append("CRITICAL: Battery extremely low!")
        elif self.battery_level <= self.LOW_BATTERY:
            alerts.append("WARNING: Battery low")
        
        # Temperature alerts
        if self.temperature >= self.CRITICAL_TEMP:
            alerts.append("CRITICAL: Temperature dangerously high!")
        elif self.temperature >= self.HIGH_TEMP:
            alerts.append("WARNING: Temperature high")
        
        # Publish alerts
        for alert in alerts:
            alert_msg = String()
            alert_msg.data = alert
            self.alert_pub.publish(alert_msg)
            self.get_logger().warn(alert)
            
            # Log to file
            self.log_to_file(alert)

    def publish_system_status(self):
        """Publish comprehensive system status"""
        status = {
            'timestamp': datetime.now().isoformat(),
            'battery_percent': round(self.battery_level, 1),
            'temperature_c': round(self.temperature, 1),
            'cpu_usage': round(self.cpu_usage, 1),
            'memory_usage': round(self.memory_usage, 1),
            'maze_metrics': {
                'clues_found': self.clues_found,
                'zones_visited': list(self.zones_visited),
                'puzzles_solved': self.puzzles_solved,
                'has_key': False  # Will be updated by puzzle logic
            },
            'system_health': self.calculate_system_health()
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.maze_status_pub.publish(status_msg)
        
        # Update simulated metrics
        self.update_simulation_metrics()

    def calculate_system_health(self):
        """Calculate overall system health score"""
        battery_score = min(100, (self.battery_level / 100.0) * 100)
        temp_score = max(0, 100 - ((self.temperature - 25) / 50.0) * 100)
        
        health = (battery_score * 0.6 + temp_score * 0.4)
        return round(max(0, min(100, health)), 1)

    def update_simulation_metrics(self):
        """Update simulation metrics (called by other nodes)"""
        # Simulate random clue discoveries
        if random.random() < 0.1:  # 10% chance each cycle
            self.clues_found += 1
        
        # Simulate zone visits
        zones = ['entry_gate', 'color_hall', 'qr_chamber', 'zone_c', 'final_exit']
        if random.random() < 0.05:  # 5% chance each cycle
            new_zone = random.choice(zones)
            self.zones_visited.add(new_zone)

    def log_to_file(self, message):
        """Log messages to file"""
        timestamp = datetime.now().isoformat()
        log_entry = f"{timestamp} - {message}\n"
        
        with open(self.log_file, 'a') as f:
            f.write(log_entry)

    def increment_clue_count(self):
        """Called when a clue is found"""
        self.clues_found += 1

    def record_zone_visit(self, zone_name):
        """Record that a zone was visited"""
        self.zones_visited.add(zone_name)

    def record_puzzle_solved(self):
        """Record that a puzzle was solved"""
        self.puzzles_solved += 1

def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
