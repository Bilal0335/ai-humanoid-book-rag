---
title: ROS 2 Architecture Layers - DDS, RMW, and Executors
sidebar_label: Architecture Layers
description: Understanding the layered architecture of ROS 2 including DDS, RMW, and executor systems for humanoid robotics
---

# ROS 2 Architecture Layers - DDS, RMW, and Executors

## Learning Objectives

- Understand the layered architecture of ROS 2 and its components
- Explain the role of DDS, RMW, and executors in the ROS 2 stack
- Identify how these layers enable distributed and real-time robotics applications
- Apply architectural concepts to humanoid robotics communication patterns
- Recognize the benefits of each layer for complex robotic systems

## Introduction

ROS 2's architecture is built on a layered approach that separates concerns and enables flexibility in implementation. Understanding these layers is crucial for humanoid robotics applications that require distributed computing, real-time performance, and reliable communication between numerous subsystems. This section explores the core architectural layers that make ROS 2 suitable for complex humanoid systems.

## The ROS 2 Architecture Stack

The ROS 2 architecture consists of several layers that work together to provide a complete robotics middleware:

```
┌─────────────────────────────────┐
│         Application Layer       │  <- Nodes, Launch files, Parameters
├─────────────────────────────────┤
│        Client Library           │  <- rclcpp, rclpy, rclnodejs, etc.
├─────────────────────────────────┤
│        RMW Layer (Middleware)   │  <- ROS Middleware Wrapper
├─────────────────────────────────┤
│        DDS Implementation       │  <- Fast DDS, Cyclone DDS, RTI Connext
└─────────────────────────────────┘
```

Each layer serves a specific purpose in the overall system:

### 1. Application Layer (Nodes and Components)

This is where your robotics applications live. For humanoid robots, this includes:
- Perception nodes (vision, lidar, IMU processing)
- Planning nodes (motion planning, path planning, manipulation planning)
- Control nodes (joint controllers, balance controllers, gait controllers)
- AI/ML nodes (behavior selection, decision making, learning systems)

### 2. Client Library Layer (rclcpp, rclpy, etc.)

The client library layer provides the familiar ROS 2 API that developers use. It handles:
- Node creation and management
- Publisher/subscriber creation
- Service/client implementation
- Parameter management
- Timer and callback handling

This layer is implemented in multiple languages (C++, Python, etc.) but provides consistent semantics across languages.

### 3. ROS Middleware (RMW) Layer

The ROS Middleware (RMW) layer acts as an abstraction layer between the client libraries and the underlying DDS implementations. It provides:
- Uniform interface to different DDS vendors
- Message type support and serialization
- Quality of Service (QoS) policy translation
- Publisher/subscriber/service abstraction

### 4. DDS Implementation Layer

The Data Distribution Service (DDS) layer provides the actual communication infrastructure:
- Discovery of participants
- Message routing and delivery
- Quality of Service enforcement
- Network communication protocols

## Data Distribution Service (DDS) in Depth

DDS is the foundation that enables ROS 2's distributed architecture. For humanoid robotics, DDS provides several critical capabilities:

### Discovery and Auto-Configuration

DDS automatically discovers participants in the network without requiring central configuration:

```
Robot Computer A           Network            Robot Computer B
┌─────────────────┐                        ┌─────────────────┐
│ Perception Node │ ◄────────────────────► │ Control Node    │
│ (Camera, IMU)   │    DDS Discovery      │ (Motors, Servos)│
└─────────────────┘                        └─────────────────┘
```

This is particularly valuable for humanoid robots that may have:
- Multiple computers for different subsystems
- Distributed processing for real-time performance
- Dynamic reconfiguration of computing resources

### Quality of Service (QoS) Profiles

DDS provides rich QoS profiles that allow fine-tuning of communication behavior:

| Profile | Use Case in Humanoid Robotics |
|---------|-------------------------------|
| RELIABLE | Joint position commands, critical safety messages |
| BEST_EFFORT | Sensor data streams, camera feeds where occasional loss is acceptable |
| DURABLE | Parameters and configuration that late-joining nodes need |
| VOLATILE | Real-time sensor data that becomes obsolete quickly |
| KEEP_LAST | Control commands where only the most recent matters |
| KEEP_ALL | Log data where all messages are important |

### Example: DDS QoS Configuration for Humanoid Control

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class HumanoidControlNode(Node):
    def __init__(self):
        super().__init__('humanoid_control')

        # High-priority control commands - RELIABLE, DURABLE, KEEP_LAST(1)
        control_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        self.joint_cmd_pub = self.create_publisher(Float64MultiArray, '/joint_commands', control_qos)

        # Sensor data - BEST_EFFORT, VOLATILE, larger buffer
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.state_callback, sensor_qos)

    def state_callback(self, msg):
        # Process joint state data
        pass
```

## ROS Middleware (RMW) Layer

The ROS Middleware (RMW) layer provides vendor-neutral access to DDS implementations. This enables:

### Middleware Abstraction

Different DDS vendors can be used interchangeably:

```
┌─────────────────┐
│   Client Lib    │  <- Same API regardless of DDS vendor
├─────────────────┤
│     RMW Layer   │  <- Abstracts DDS differences
├─────────────────┤
│   Fast DDS      │  <- One possible implementation
│   Cyclone DDS   │  <- Another possible implementation
│   RTI Connext   │  <- Yet another implementation
│   Others...     │
└─────────────────┘
```

### Benefits for Humanoid Robotics

1. **Vendor Independence**: Switch between DDS implementations without changing application code
2. **Performance Tuning**: Choose DDS implementation based on performance requirements
3. **Certification**: Select certified DDS implementations for safety-critical applications
4. **Resource Optimization**: Choose lightweight implementations for embedded systems

## Executors in ROS 2

Executors manage the execution of callbacks in ROS 2 nodes. Understanding executors is crucial for humanoid robotics where real-time performance is essential.

### Single-threaded Executor

```python
executor = SingleThreadedExecutor()
executor.add_node(node)
executor.spin()
```

- All callbacks run in a single thread
- Simple and predictable behavior
- May cause blocking if callbacks are slow
- Good for simple nodes or debugging

### Multi-threaded Executor

```python
executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(node)
executor.spin()
```

- Callbacks can run in parallel across multiple threads
- Better performance for I/O-heavy applications
- Potential for race conditions
- Good for nodes with many I/O operations

### Custom Executor Patterns for Humanoid Robotics

For humanoid robots, you might want custom executor patterns:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.executors import Executor, Future
from rclpy.task import Task
from threading import Thread
import time

class RealTimeHumanoidExecutor(Executor):
    """
    Custom executor optimized for real-time humanoid control
    """
    def __init__(self):
        super().__init__()
        self.control_thread = None
        self.perception_thread = None

    def spin_once(self, timeout_sec=None):
        # Separate time-critical control callbacks from perception callbacks
        # This ensures control loops aren't blocked by perception processing
        pass

class HumanoidNodeWithRealTimeExecutor:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('humanoid_realtime_node')

        # Time-critical control timer (1kHz)
        self.control_timer = self.node.create_timer(0.001, self.control_callback,
                                                   callback_group=rclpy.callback_groups.ReentrantCallbackGroup())

        # Less critical perception timer (30Hz)
        self.perception_timer = self.node.create_timer(0.033, self.perception_callback,
                                                      callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup())
```

## Practical Architecture for Humanoid Robotics

### Example: Humanoid Robot Communication Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     Humanoid Robot System                       │
├─────────────────────────────────────────────────────────────────┤
│ Computer 1: Perception                                          │
│ ┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐     │
│ │  Camera Driver  │ │   IMU Driver    │ │  Lidar Driver   │     │
│ │     (ROS 2)     │ │    (ROS 2)     │ │    (ROS 2)     │     │
│ └─────────────────┘ └─────────────────┘ └─────────────────┘     │
│           │                      │                    │        │
│           └──────────────────────┼────────────────────┘        │
│                                  │                             │
│ ┌─────────────────────────────────────────────────────────┐   │
│ │                    Perception Node                      │   │
│ │                    (ROS 2 Node)                        │   │
│ └─────────────────────────────────────────────────────────┘   │
│                                  │                             │
└──────────────────────────────────┼─────────────────────────────┘
                                   │
┌──────────────────────────────────┼─────────────────────────────┐
│                                 ▼                             │
│ ┌─────────────────────────────────────────────────────────┐   │
│ │                   ROS 2 Communication Layer           │   │
│ │                   (DDS/RMW/Client Library)            │   │
│ └─────────────────────────────────────────────────────────┘   │
│                                 │                             │
│ ┌─────────────────────────────────────────────────────────┐   │
│ │                    Planning Node                        │   │
│ │                    (Motion/Path Planning)               │   │
│ └─────────────────────────────────────────────────────────┘   │
│                                 │                             │
│ ┌─────────────────────────────────────────────────────────┐   │
│ │                    Control Computer                     │   │
│ │  ┌─────────────────┐ ┌─────────────────┐               │   │
│ │  │ Balance Control │ │ Joint Control   │               │   │
│ │  │     Node        │ │    Node         │               │   │
│ │  └─────────────────┘ └─────────────────┘               │   │
│ └─────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

### DDS Configuration for Humanoid Robotics

For humanoid robots, specific DDS configurations are beneficial:

#### Real-time Performance Settings

```xml
<!-- In DDS configuration file -->
<dds>
    <profiles>
        <!-- Profile for critical control data -->
        <data_writer_qos_profile name="control_qos">
            <reliability>
                <kind>RELIABLE_RELIABILITY_QOS</kind>
            </reliability>
            <durability>
                <kind>VOLATILE_DURABILITY_QOS</kind>
            </durability>
            <history>
                <kind>KEEP_LAST_HISTORY_QOS</kind>
                <depth>1</depth>
            </history>
            <publish_mode>
                <kind>ASYNCHRONOUS_PUBLISH_MODE_QOS</kind>
            </publish_mode>
        </data_writer_qos_profile>

        <!-- Profile for sensor data -->
        <data_reader_qos_profile name="sensor_qos">
            <reliability>
                <kind>BEST_EFFORT_RELIABILITY_QOS</kind>
            </reliability>
            <durability>
                <kind>VOLATILE_DURABILITY_QOS</kind>
            </durability>
            <history>
                <kind>KEEP_LAST_HISTORY_QOS</kind>
                <depth>10</depth>
            </history>
        </data_reader_qos_profile>
    </profiles>
</dds>
```

## Advanced Architecture Considerations

### Domain Isolation

DDS domains provide isolation between different systems:

```python
# Set environment variable to isolate robot systems
# export ROS_DOMAIN_ID=42  # For specific robot or lab

# This allows multiple robots to operate in the same network
# without interfering with each other's communications
```

### Security Configuration

For humanoid robots operating in human environments:

```xml
<!-- DDS Security configuration -->
<dds>
    <security>
        <authentication>
            <library>dds::security::builtin_authentication</library>
            <properties>
                <property name="dds.sec.auth.plugin">builtin.SHA256</property>
                <property name="dds.sec.auth.builtin.SHA256.private_key">private_key.pem</property>
                <property name="dds.sec.auth.builtin.SHA256.identity_ca">identity_ca.pem</property>
            </properties>
        </authentication>
        <access_control>
            <library>dds::security::builtin_access_control</library>
            <properties>
                <property name="dds.sec.access.plugin">builtin.Access-Permissions</property>
                <property name="dds.sec.access.builtin.Access-Permissions.governance">governance.p7s</property>
                <property name="dds.sec.access.builtin.Access-Permissions.permissions">permissions.p7s</property>
            </properties>
        </access_control>
    </security>
</dds>
```

## Key Takeaways

- DDS provides the foundation for distributed, real-time robotics communication
- RMW enables middleware vendor independence and flexibility
- Executors control how callbacks are processed and affect real-time performance
- QoS profiles allow fine-tuning communication behavior for different robot subsystems
- Proper architecture selection is crucial for humanoid robotics performance and reliability

## Next Steps

- Experiment with different QoS profiles for various humanoid robotics communication patterns
- Set up a multi-computer ROS 2 network for distributed humanoid control
- Configure DDS for optimal performance in your specific humanoid application
- Implement custom executors for time-critical control loops

## References

- ROS 2 Architecture: https://docs.ros.org/en/humble/Concepts/Architecture-Overview.html
- Quality of Service: https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html
- DDS Standard: https://www.omg.org/spec/DDS/
- RMW Interface: https://github.com/ros2/rmw