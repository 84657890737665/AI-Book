# Quickstart Guide: Physical AI & Humanoid Robotics Hackathon

## Overview
This guide will help you set up your development environment and begin working on your Physical AI & Humanoid Robotics Hackathon project. The hackathon focuses on creating an embodied AI system that bridges digital AI models with physical robotics platforms.

## Prerequisites

### Hardware Requirements
- NVIDIA RTX GPU (4090/4080 recommended) for Isaac Sim
- NVIDIA Jetson Orin Nano/NX for physical deployment (if using real robot)
- If humanoid unavailable, prepare alternative platform:
  - Unitree Go2 (quadruped)
  - Robotic arm
  - Miniature humanoid (Hiwonder, OP3)

### Software Requirements
- Ubuntu 22.04 LTS (recommended for consistency)
- ROS 2 Humble Hawksbill or Iron Irwini
- Python 3.10+ with pip
- Git version control

## Environment Setup

### 1. Install ROS 2
```bash
# Add ROS 2 repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions
```

### 2. Install Isaac Sim (Optional, for advanced teams)
- Download Isaac Sim from NVIDIA Developer website
- Follow installation instructions for your RTX GPU
- Verify installation with provided sample scenes

### 3. Install Gazebo (Alternative simulation)
```bash
sudo apt install ros-humble-gazebo-*
```

### 4. Set up Virtual Environment
```bash
# Create virtual environment
python3 -m venv ~/robot_env
source ~/robot_env/bin/activate

# Install Python dependencies
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip install transformers openai-whisper opencv-python
```

## Project Structure

After setup, your project should have the following structure:

```
robot_system/
├── src/
│   ├── communication/       # ROS 2 nodes, topics, services, actions
│   ├── perception/          # VSLAM and computer vision modules
│   ├── control/             # Robot motion and manipulation control
│   ├── planning/            # AI model integration and action planning
│   ├── simulation/          # Gazebo/Isaac Sim integration
│   └── voice_interface/     # Speech recognition and voice processing
├── config/
├── launch/
├── tests/
└── scripts/
```

## Getting Started: Basic Communication Pipeline

### 1. Create Your Package
```bash
source /opt/ros/humble/setup.bash
mkdir -p robot_system/src
cd robot_system
colcon build
source install/setup.bash

# Create robot communication package
ros2 pkg create --build-type ament_python robot_communication
```

### 2. Implement Basic Publisher-Subscriber
Create a simple node in `robot_system/src/robot_communication/robot_communication/talker.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'robot_commands', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello Robot! Time: %s' % self.get_clock().now().to_msg()
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    talker = Talker()
    
    try:
        rclpy.spin(talker)
    except KeyboardInterrupt:
        pass
    finally:
        talker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Run Your Basic System
```bash
cd robot_system
source install/setup.bash
python3 src/robot_communication/robot_communication/talker.py
```

## Next Steps for the Hackathon

1. **Implement Perception Module**: Work on VSLAM or computer vision capabilities
2. **Build Voice Interface**: Create the speech recognition to action planning pipeline
3. **Integrate Simulation**: Set up Gazebo or Isaac Sim environment
4. **Connect to Real Robot**: If available, connect to your physical platform
5. **Implement Task Execution**: Create end-to-end task completion

## Key Requirements to Focus On

- Response latency under 200ms (FR-011, SC-016)
- Basic security measures (FR-012, SC-017)
- Fallback mechanisms for external APIs (FR-013, SC-018)
- Basic logging (FR-014, SC-019)
- Basic state management (FR-015, SC-020)

## Testing Your Implementation

### Unit Testing
```bash
# Run Python unit tests
python3 -m pytest tests/unit/
```

### Integration Testing
```bash
# Run ROS 2 integration tests
source install/setup.bash
colcon test --packages-select robot_communication
```

### Simulation Testing
```bash
# Launch simulation environment
source install/setup.bash
ros2 launch robot_system simulation.launch.py
```

## Troubleshooting Common Issues

### ROS 2 Communication Issues
- Check that all nodes are on the same ROS_DOMAIN_ID
- Verify network configuration for multi-machine setups
- Use `ros2 topic list` to confirm topics are being created

### Performance Issues
- Monitor CPU and memory usage on Jetson devices
- Consider optimizing image processing pipelines
- Use intra-process communication where possible

### Simulation Issues
- Ensure GPU drivers are properly installed
- Check Isaac Sim or Gazebo logs for specific errors
- Verify robot model URDF files are correctly formatted

## Resources

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [NVIDIA Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/what_is_isaac_sim.html)
- [Gazebo Documentation](http://gazebosim.org/)
- [PyTorch Documentation](https://pytorch.org/docs/stable/index.html)

## Evaluation Criteria

Your project will be evaluated based on:
1. Technical difficulty (simulation fidelity, perception quality, locomotion control)
2. Integration depth (ROS 2 + Isaac + VLA + sensors + planning)
3. Stability of the system during demo (no catastrophic failures)
4. Creativity of task selection and robot behavior
5. Clarity of documentation + reproducibility

Good luck with your hackathon project!