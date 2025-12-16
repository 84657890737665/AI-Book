# Data Model: Physical AI & Humanoid Robotics Hackathon

## Entities

### 1. Robot Communication Pipeline
**Description**: The communication framework connecting robot nodes, topics, services, and actions
**Attributes**:
- pipeline_id (string, unique identifier)
- nodes (array of Node references)
- topics (array of Topic references)
- services (array of Service references)
- actions (array of Action references)
- status (enum: "active", "inactive", "error")
- created_timestamp (datetime)
- last_heartbeat (datetime)

**Relationships**:
- Contains multiple Node entities
- Communicates via Topic entities
- Provides multiple Service entities
- Offers multiple Action entities

**Validation Rules**:
- pipeline_id must be unique
- must have at least one node
- all connections must be valid

### 2. Node
**Description**: Individual ROS 2 node within the communication pipeline
**Attributes**:
- node_id (string, unique identifier)
- node_name (string, ROS 2 compatible name)
- node_type (enum: "perception", "control", "planning", "voice_interface", "simulation")
- status (enum: "running", "idle", "error", "shutdown")
- cpu_usage (float, percentage)
- memory_usage (float, in MB)
- last_heartbeat (datetime)

**Relationships**:
- Belongs to one Robot Communication Pipeline
- Publishes to multiple Topic entities
- Subscribes to multiple Topic entities
- Provides multiple Service entities
- Offers multiple Action entities

**Validation Rules**:
- node_name must follow ROS 2 naming conventions
- node_id must be unique within the pipeline

### 3. Topic
**Description**: Communication channel for publishing/subscribing messages in the ROS 2 system
**Attributes**:
- topic_id (string, unique identifier)
- topic_name (string, ROS 2 compatible name)
- message_type (string, type definition)
- publisher_count (integer, number of publishers)
- subscriber_count (integer, number of subscribers)
- frequency (float, in Hz)
- qos_profile (string, Quality of Service settings)

**Relationships**:
- Belongs to one or more Node entities (publishers)
- Belongs to one or more Node entities (subscribers)

**Validation Rules**:
- topic_name must follow ROS 2 naming conventions
- frequency must be positive
- message_type must be valid ROS 2 message type

### 4. Service
**Description**: Synchronous request/response communication mechanism in the ROS 2 system
**Attributes**:
- service_id (string, unique identifier)
- service_name (string, ROS 2 compatible name)
- request_type (string, request message type)
- response_type (string, response message type)
- status (enum: "available", "busy", "unavailable")
- timeout (float, in seconds)

**Relationships**:
- Belongs to one Node entity (the service provider)

**Validation Rules**:
- service_name must follow ROS 2 naming conventions
- timeout must be positive

### 5. Action
**Description**: Asynchronous goal-based communication for long-running tasks
**Attributes**:
- action_id (string, unique identifier)
- action_name (string, ROS 2 compatible name)
- goal_type (string, goal message type)
- result_type (string, result message type)
- feedback_type (string, feedback message type)
- status (enum: "idle", "active", "preempted", "succeeded", "aborted", "rejected")
- timeout (float, in seconds)

**Relationships**:
- Belongs to one Node entity

**Validation Rules**:
- action_name must follow ROS 2 naming conventions
- timeout must be positive

### 6. Simulated Robot Environment
**Description**: Physics simulation platform for robot motion demonstration
**Attributes**:
- environment_id (string, unique identifier)
- environment_name (string, descriptive name)
- simulation_type (enum: "gazebo", "isaac_sim", "unity", "custom")
- physics_engine (string, engine name and version)
- robot_model (string, URDF or similar model reference)
- world_file (string, path to simulation world)
- status (enum: "running", "paused", "stopped")
- simulation_time (float, seconds since start)

**Relationships**:
- Contains one Physical Robot Platform simulation
- Connected to Robot Communication Pipeline for control

**Validation Rules**:
- environment_name must be unique
- simulation_type must be one of the supported types
- world_file must exist and be valid

### 7. Perception Module
**Description**: Spatial mapping or computer vision system for environmental awareness
**Attributes**:
- module_id (string, unique identifier)
- module_type (enum: "vslam", "computer_vision", "lidar", "other")
- input_source (string, sensor or data stream reference)
- output_format (string, format of perception data)
- processing_rate (float, in Hz)
- accuracy_metric (float, measure of perception accuracy)
- status (enum: "active", "idle", "error")

**Relationships**:
- Belongs to one Robot Communication Pipeline
- Connected to Sensor entities
- Publishes to Topic entities with perception data

**Validation Rules**:
- processing_rate must be positive
- accuracy_metric must be between 0 and 1 for normalized metrics

### 8. Voice-to-Action Pipeline
**Description**: System component converting speech to robot actions via AI model processing
**Attributes**:
- pipeline_id (string, unique identifier)
- input_source (string, audio input device or stream)
- speech_model (string, speech recognition model used)
- ai_model (string, AI model used for action planning)
- output_format (string, action command format)
- processing_latency (float, in milliseconds)
- accuracy_metric (float, measure of voice recognition accuracy)
- status (enum: "active", "idle", "error")

**Relationships**:
- Belongs to one Robot Communication Pipeline
- Connected to Topic entities for command output
- Connected to Action entities for complex commands

**Validation Rules**:
- processing_latency must be under 200ms (as per FR-011)
- accuracy_metric must be between 0 and 1 for normalized metrics

### 9. Physical Robot Platform
**Description**: Hardware robot platform (humanoid or proxy) for task execution
**Attributes**:
- platform_id (string, unique identifier)
- platform_type (enum: "humanoid", "quadruped", "arm", "wheeled", "other")
- manufacturer (string, robot manufacturer)
- model (string, robot model)
- joint_count (integer, number of controllable joints)
- sensor_count (integer, number of sensors)
- computational_capacity (string, CPU/GPU specs)
- battery_life (float, in minutes)
- status (enum: "connected", "disconnected", "error", "charging")
- current_task (string, description of current task)

**Relationships**:
- Connected to Robot Communication Pipeline
- Contains multiple Sensor entities
- Contains multiple Actuator entities

**Validation Rules**:
- joint_count must be positive
- sensor_count must be non-negative

### 10. Edge Computing Device
**Description**: Hardware for real-time processing on physical robots
**Attributes**:
- device_id (string, unique identifier)
- device_type (enum: "jetson_orin_nano", "jetson_orin_nx", "other")
- manufacturer (string, device manufacturer)
- model (string, device model)
- cpu_cores (integer, number of CPU cores)
- gpu_type (string, GPU model)
- memory (integer, in GB)
- storage (integer, in GB)
- power_consumption (float, in watts)
- temperature (float, current temperature in Celsius)
- status (enum: "active", "idle", "error", "overheating")

**Relationships**:
- Connected to Physical Robot Platform
- Runs Robot Communication Pipeline components

**Validation Rules**:
- memory and storage must be positive
- temperature must be within safe operating range

### 11. Sensor
**Description**: Individual sensor on the robot platform
**Attributes**:
- sensor_id (string, unique identifier)
- sensor_type (enum: "camera", "lidar", "imu", "accelerometer", "gyroscope", "microphone", "other")
- topic_output (string, ROS 2 topic for sensor data)
- sampling_rate (float, in Hz)
- resolution (string, sensor resolution)
- status (enum: "active", "inactive", "error")
- calibration_status (enum: "calibrated", "needs_calibration", "uncalibrated")

**Relationships**:
- Belongs to Physical Robot Platform or Perception Module
- Publishes to Topic entities

**Validation Rules**:
- sampling_rate must be positive
- sensor_type must be one of the supported types

### 12. Actuator
**Description**: Individual actuator on the robot platform
**Attributes**:
- actuator_id (string, unique identifier)
- actuator_type (enum: "servo", "motor", "hydraulic", "pneumatic", "other")
- control_topic (string, ROS 2 topic for control commands)
- position_limits (array of float, min and max positions in radians or degrees)
- velocity_limits (float, max velocity)
- effort_limits (float, max effort/torque)
- status (enum: "active", "inactive", "error")

**Relationships**:
- Belongs to Physical Robot Platform
- Subscribes to Topic entities for control commands

**Validation Rules**:
- position_limits must have exactly 2 values with min ≤ max
- velocity_limits and effort_limits must be positive

## State Transitions

### Robot Platform States
- Initial → Connected (when platform connects to communication pipeline)
- Connected → Operational (when all systems are ready)
- Operational → Busy (when executing a task)
- Busy → Operational (when task completes successfully)
- Busy → Operational (when task fails)
- Any state → Error (when error occurs)
- Error → Connected (when error is resolved)
- Any state → Disconnected (when platform disconnects)

### Action Execution States
- Idle → Active (when goal is accepted)
- Active → Succeeded (when goal reaches goal state)
- Active → Aborted (when goal is cancelled or times out)
- Active → Rejected (when goal is invalid)
- Active → Preempted (when new goal preempts current goal)

## Relationships Summary

The data model forms a hierarchical structure:
- Robot Communication Pipeline contains Nodes, which communicate via Topics, Services and Actions
- Physical Robot Platform contains Sensors and Actuators
- Perception Module processes input from Sensors
- Voice-to-Action Pipeline connects speech input to action commands
- Simulated Robot Environment simulates the Physical Robot Platform
- Edge Computing Device hosts the computation for the various modules