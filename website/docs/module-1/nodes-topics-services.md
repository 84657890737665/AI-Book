---
sidebar_position: 2
---

# Nodes, Topics, Services, and Actions

## Core ROS 2 Architecture

The ROS 2 communication system is built around four main concepts:

- **Nodes**: Basic compute elements of a ROS 2 system
- **Topics**: Named buses for message passing between nodes
- **Services**: Synchronous request/response communication
- **Actions**: Asynchronous, goal-oriented communication

## Nodes

Nodes are the fundamental building blocks of a ROS 2 program. They perform the actual work of the robot by executing algorithms, controlling hardware, or processing sensor data.

```python
import rclpy
from rclpy.node import Node

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('my_robot_node')
        self.get_logger().info('My Robot Node Created')
```

## Topics and Publishers/Subscribers

Topics enable asynchronous communication between nodes using a publish/subscribe model.

```python
# Publisher
publisher = self.create_publisher(String, 'topic_name', 10)

# Subscriber  
subscriber = self.create_subscription(
    String,
    'topic_name',
    self.topic_callback,
    10
)
```

## Services

Services provide synchronous request/response communication.

```python
# Service Server
self.service = self.create_service(
    AddTwoInts, 
    'add_two_ints', 
    self.add_two_ints_callback
)

# Service Client
self.client = self.create_client(AddTwoInts, 'add_two_ints')
```

## Actions

Actions are designed for long-running tasks with feedback and status updates.

```python
# Action Server
self._action_server = ActionServer(
    self,
    Fibonacci,
    'fibonacci',
    self.execute_callback
)

# Action Client
self._action_client = ActionClient(self, Fibonacci, 'fibonacci')
```

## Application in Physical AI

These communication patterns are essential for:

- **Sensor data distribution**: Topics for sensor streams
- **Control commands**: Services for immediate actions
- **Task execution**: Actions for complex behaviors
- **System monitoring**: Topics for status updates