---
sidebar_position: 3
---

# rclpy Python Agents

## Python in ROS 2

Python is one of the primary languages supported by ROS 2 through the rclpy package. This makes it accessible for rapid prototyping and integration with AI frameworks that often have Python APIs.

## rclpy Overview

rclpy is a Python client library for ROS 2 that provides:

- Node creation and management
- Publisher and subscriber functionality
- Service and action interfaces
- Parameter handling
- Time and duration utilities

## Basic rclpy Node Structure

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Common message types

class RobotAgent(Node):
    def __init__(self):
        super().__init__('robot_agent')
        
        # Create publisher
        self.publisher_ = self.create_publisher(String, 'robot_status', 10)
        
        # Create subscriber
        self.subscription = self.create_subscription(
            String,
            'robot_commands',
            self.command_callback,
            10
        )
        
        # Create timer for periodic tasks
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        self.i = 0

    def command_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
        
    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from robot agent: {self.i}'
        self.publisher_.publish(msg)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    robot_agent = RobotAgent()
    
    try:
        rclpy.spin(robot_agent)
    except KeyboardInterrupt:
        pass
    finally:
        robot_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration with AI Libraries

One of the key advantages of using Python with ROS 2 is seamless integration with AI libraries:

```python
import rclpy
from rclpy.node import Node
import torch  # PyTorch for AI model inference
import cv2    # OpenCV for computer vision
import numpy as np

class AIAgent(Node):
    def __init__(self):
        super().__init__('ai_agent')
        
        # Initialize AI model
        self.model = torch.load('path_to_model.pt')
        self.model.eval()
        
        # Create subscribers for sensor data
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )
        
        # Create publishers for AI decisions
        self.action_pub = self.create_publisher(String, 'ai_actions', 10)
        
    def image_callback(self, msg):
        # Convert ROS image to OpenCV format
        image = self.ros_to_cv2(msg)
        
        # Process with AI model
        with torch.no_grad():
            input_tensor = self.preprocess_image(image)
            prediction = self.model(input_tensor)
            
        # Publish AI decision
        action_msg = String()
        action_msg.data = self.decode_prediction(prediction)
        self.action_pub.publish(action_msg)
```

## Best Practices

- Use appropriate QoS profiles for your robot application
- Implement proper error handling for robust operation
- Consider threading for computationally expensive processes
- Use lifecycle nodes for complex initialization sequences
- Properly handle node destruction and resource cleanup