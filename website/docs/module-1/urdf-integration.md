---
sidebar_position: 4
---

# URDF Integration

## Unified Robot Description Format (URDF)

URDF (Unified Robot Description Format) is an XML format for representing a robot model. It defines the physical and visual properties of a robot, including:

- Links: Rigid parts of the robot
- Joints: Connections between links
- Inertial properties: Mass, center of mass, and inertia
- Visual properties: How the robot appears in simulation
- Collision properties: How the robot interacts with the environment

## URDF in Physical AI

URDF is crucial for:
- **Simulation**: Creating accurate robot models for testing
- **Visualization**: Displaying robots in Rviz and other tools
- **Collision detection**: Preventing robot self-collisions
- **Kinematic calculations**: Forward and inverse kinematics
- **Physics simulation**: Accurate physics behavior in simulators

## Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- A wheel link connected via a joint -->
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0.2 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## Integration with ROS 2

In ROS 2, URDF models are typically loaded and published using the robot_state_publisher:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf2_ros
import tf2_geometry_msgs
from urdf_parser_py.urdf import URDF

class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')
        
        # Load URDF
        self.robot_description = self.declare_parameter(
            'robot_description',
            '').value
            
        # Create transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer to broadcast transforms
        self.timer = self.create_timer(0.05, self.publish_transforms)

    def publish_transforms(self):
        # Calculate and publish joint transforms
        # (Implementation depends on your robot's kinematics)
        pass
```

## URDF for Physical AI Applications

For Physical AI robots specifically, consider:

- **Sensor mounting**: Define precise locations for cameras, LiDAR, etc.
- **End-effector definition**: For manipulation tasks
- **Collision meshes**: Balance accuracy with performance
- **Material properties**: For physics simulation
- **Transmission elements**: For joint control

## Tools for URDF Development

- **RViz**: Visualize your robot model
- **Robot Web Tools**: Browser-based visualization
- **Gazebo/Mujoco/Isaac Sim**: Physics simulation with your robot model
- **Xacro**: Macro system for URDF to reduce repetition
- **URDF tutorials**: In-depth examples and best practices