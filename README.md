# escape maze

# Prerequisites
System Requirements
Ubuntu 22.04 (Recommended) or Ubuntu 20.04
ROS2 Humble (for Ubuntu 22.04) or ROS2 Foxy (for Ubuntu 20.04)
Python 3.8+
Gazebo Classic or Gazebo Fortress
8GB RAM minimum, 16GB recommended 

# Required ROS2 Packages
sudo apt install ros-$ROS_DISTRO-gazebo-ros
sudo apt install ros-$ROS_DISTRO-turtlebot3-gazebo
sudo apt install ros-$ROS_DISTRO-nav2-bringup
sudo apt install ros-$ROS_DISTRO-cv-bridge
sudo apt install ros-$ROS_DISTRO-vision-opencv
sudo apt install ros-$ROS_DISTRO-rviz2

Python Dependencies
pip install opencv-python numpy ultralytics PyYAML
# Optional for voice control:
pip install SpeechRecognition pyaudio

Clone or Create Workspace
mkdir -p ~/escape_maze_ws/src
cd ~/escape_maze_ws/src

# Create package structure
ros2 pkg create escape_maze --build-type ament_python --dependencies rclpy std_msgs geometry_msgs sensor_msgs nav_msgs nav2_msgs cv_bridge

Copy files to package
# Copy all Python files to the package directory
cp *.py ~/escape_maze_ws/src/escape_maze/escape_maze/

# Copy launch, config, and world files
mkdir -p ~/escape_maze_ws/src/escape_maze/launch
mkdir -p ~/escape_maze_ws/src/escape_maze/config
mkdir -p ~/escape_maze_ws/src/escape_maze/worlds
cp *.launch.py ~/escape_maze_ws/src/escape_maze/launch/
cp *.yaml ~/escape_maze_ws/src/escape_maze/config/
cp *.world ~/escape_maze_ws/src/escape_maze/worlds/

Build Package
cd ~/escape_maze_ws
colcon build --packages-select escape_maze
source install/setup.bash

Set-Up Environment Variables

echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/escape_maze_ws/src/escape_maze/models" >> ~/.bashrc
source ~/.bashrc


# Running the Simulation

Terminal 1: Launch complete simulation
ros2 launch escape_maze escape_maze_launch.py

Terminal 1: Gazebo simulation
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

Terminal 2: Navigation stack
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True

Terminal 3: Maze control system
ros2 run escape_maze maze_controller
ros2 run escape_maze puzzle_logic_engine
ros2 run escape_maze clue_detector

Terminal 4: Voice control (optional)
ros2 run escape_maze voice_emergency_stop


# Testing Commands
Test shape clue
ros2 topic pub /maze/clue_detected std_msgs/String '{"data": "{\"type\": \"shape\", \"value\": \"triangle\"}"}'

Test color clue
ros2 topic pub /maze/clue_detected std_msgs/String '{"data": "{\"type\": \"color\", \"value\": \"red\"}"}'

Test QR clue
ros2 topic pub /maze/clue_detected std_msgs/String '{"data": "{\"type\": \"qr\", \"value\": \"Proceed to Zone 3\"}"}'

# Monitor the System

# View all active topics
ros2 topic list

# Monitor clue detection
ros2 topic echo /maze/clue_detected

# Monitor navigation status
ros2 topic echo /navigation/status

# Monitor system health
ros2 topic echo /maze/system_status

# View node graph
rqt_graph


# Achievements

When the system works correctly, you should see:
Robot autonomously navigating through maze zones
Successful clue detection and interpretation
Puzzle solving leading to key acquisition
Final escape from the maze
Victory message in console and RViz

# Limitations

Simulation performance depends on hardware
Voice control requires microphone in real mode
YOLO detection accuracy depends on training data
Maze complexity limited by Gazebo performance
