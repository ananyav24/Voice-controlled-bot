# Voice Controlled Robot using ROS2
This ROS2 package allows a user to control a robot in Gazebo simulation using Voice commands. 

## Features
- Voice recognition using microphone
- Commands supported: "forward", "stop", "left", and "right". 
- Uses Google Speech Recognition API
- Publishes the String commands to a ROS2 topic (/voice_cmd)
- Subscriber converts words to geometry_msgs/Twist.
- Gazebo robot subscribes and moves based on the voice command.

## Requirements
- ROS2 Humble 
- Python Libraries: 
```bash 
pip3 install SpeechRecognition PyAudio
sudo apt install portaudio19-dev
```
- ROS2 Packages:
```bash
rclpy
std_msgs
gazebo_ros
```
### Build
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```
### Run
1. Start the Gazebo simulation
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
2. Start the voice command publisher
```bash
python3 ~/ros2_ws/src/voice_robot/voice_robot/voice_publisher.py
```
3. Start the voice command subscriber
```bash
python3 ~/ros2_ws/src/voice_robot/voice_robot/voice_subscriber.py
```
4. Check the topic
```bash
ros2 topic echo /voice_cmd
```
Example messages:
```bash
data: "forward"
data: "left"
```

## Demo
yet to add
