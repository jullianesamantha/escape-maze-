
#!/usr/bin/env python3
"""
Voice Activated Emergency Stop for Escape Maze
Enhanced with simulation support and maze-specific commands
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import threading
import json

class VoiceEmergencyStop(Node):
    def __init__(self):
        super().__init__('voice_emergency_stop')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.emergency_pub = self.create_publisher(String, '/emergency/stop', 10)
        
        # Subscriber for simulated voice commands (for simulation)
        self.create_subscription(String, '/voice/simulated_command', 
                                self.sim_command_callback, 10)
        
        # Maze-specific voice commands
        self.maze_commands = {
            'stop': self.emergency_stop,
            'pause': self.pause_robot,
            'continue': self.continue_robot,
            'help': self.voice_help,
            'status': self.voice_status
        }
        
        # Voice recognition setup (optional - for real hardware)
        self.use_voice = False  # Set to False for pure simulation
        
        if self.use_voice:
            self.init_voice_recognition()
        
        self.get_logger().info("Voice Emergency Stop initialized")
        self.get_logger().info("Simulation mode: Use /voice/simulated_command topic")
        self.get_logger().info("Commands: stop, pause, continue, help, status")

    def init_voice_recognition(self):
        """Initialize voice recognition (for real hardware)"""
        try:
            import speech_recognition as sr
            self.recognizer = sr.Recognizer()
            self.mic = sr.Microphone()
            
            # Start listening thread
            self.listening = True
            self.voice_thread = threading.Thread(target=self.voice_listen_loop)
            self.voice_thread.daemon = True
            self.voice_thread.start()
            
            self.get_logger().info("Voice recognition active - Say 'stop' to halt robot")
        except ImportError:
            self.get_logger().warn("SpeechRecognition not installed. Voice disabled.")
        except Exception as e:
            self.get_logger().error(f"Voice init failed: {e}")

    def voice_listen_loop(self):
        """Voice listening thread (for real hardware)"""
        import speech_recognition as sr
        
        with self.mic as source:
            self.recognizer.adjust_for_ambient_noise(source)
            
            while self.listening and rclpy.ok():
                try:
                    audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=3)
                    text = self.recognizer.recognize_google(audio).lower()
                    
                    self.get_logger().info(f"Heard: {text}")
                    self.process_voice_command(text)
                    
                except sr.WaitTimeoutError:
                    continue
                except sr.UnknownValueError:
                    continue
                except Exception as e:
                    self.get_logger().error(f"Voice error: {e}")
                    break

    def sim_command_callback(self, msg):
        """Process simulated voice commands"""
        try:
            command_data = json.loads(msg.data)
            command = command_data.get('command', '').lower()
            
            self.get_logger().info(f"Simulated voice command: {command}")
            self.process_voice_command(command)
            
        except json.JSONDecodeError:
            # Plain text command
            self.process_voice_command(msg.data.lower())

    def process_voice_command(self, command_text):
        """Process voice command text"""
        # Check for maze commands
        for cmd_keyword, cmd_func in self.maze_commands.items():
            if cmd_keyword in command_text:
                cmd_func()
                return
        
        # Check for emergency stop synonyms
        stop_words = ['halt', 'emergency', 'abort', 'freeze']
        if any(word in command_text for word in stop_words):
            self.emergency_stop()

    def emergency_stop(self):
        """Emergency stop - halt all robot movement"""
        # Publish zero velocity
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        
        for _ in range(3):  # Publish multiple times for reliability
            self.cmd_vel_pub.publish(twist)
        
        # Publish emergency message
        emergency_msg = String()
        emergency_msg.data = "VOICE_EMERGENCY_STOP"
        self.emergency_pub.publish(emergency_msg)
        
        self.get_logger().warn("EMERGENCY STOP ACTIVATED VIA VOICE COMMAND")
        
        # Flash warning (simulated)
        warning_msg = String()
        warning_msg.data = "Robot halted by voice command"
        self.emergency_pub.publish(warning_msg)

    def pause_robot(self):
        """Pause robot movement temporarily"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        self.get_logger().info("Robot paused by voice command")
        
        status_msg = String()
        status_msg.data = "PAUSED_BY_VOICE"
        self.emergency_pub.publish(status_msg)

    def continue_robot(self):
        """Resume robot operation"""
        self.get_logger().info("Continue command received")
        
        status_msg = String()
        status_msg.data = "RESUMED_BY_VOICE"
        self.emergency_pub.publish(status_msg)

    def voice_help(self):
        """Provide voice help"""
        help_text = "Available commands: stop, pause, continue, status"
        self.get_logger().info(f"Voice help: {help_text}")
        
        help_msg = String()
        help_msg.data = help_text
        self.emergency_pub.publish(help_msg)

    def voice_status(self):
        """Report robot status"""
        status_msg = String()
        status_msg.data = "VOICE_CONTROL_ACTIVE"
        self.emergency_pub.publish(status_msg)
        
        self.get_logger().info("Status requested via voice")

def main(args=None):
    rclpy.init(args=args)
    node = VoiceEmergencyStop()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.listening = False
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()