---
title: ROS 1 vs ROS 2 - Middleware Evolution for Humanoid Robotics
sidebar_label: ROS 1 vs ROS 2
description: Comparing the evolution from ROS 1 to ROS 2 and its impact on humanoid robotics applications
---

# ROS 1 vs ROS 2 - Middleware Evolution for Humanoid Robotics

## Learning Objectives

- Compare the fundamental differences between ROS 1 and ROS 2
- Understand why ROS 2 is better suited for humanoid robotics applications
- Identify the key improvements that make ROS 2 more robust and reliable
- Recognize the architectural changes that enable distributed and secure robotics systems

## Introduction

The transition from ROS 1 to ROS 2 represents one of the most significant evolutions in robotics middleware development. For humanoid robotics applications, which demand high reliability, real-time performance, and distributed computing capabilities, these changes are particularly impactful. This section explores the key differences between ROS 1 and ROS 2 and explains why ROS 2 is essential for modern humanoid robotics.

## Fundamental Architecture Differences

### Master-Based vs. DDS-Based Architecture

**ROS 1 Architecture:**
- Centralized master node that manages all communication
- All nodes register with the master to discover each other
- Single point of failure - if the master goes down, the entire system fails
- Limited support for distributed systems across networks

**ROS 2 Architecture:**
- Decentralized architecture based on Data Distribution Service (DDS)
- Nodes discover each other directly using DDS protocols
- No single point of failure - nodes can continue operating even if others fail
- Native support for distributed systems across networks and computers

For humanoid robots with dozens of nodes running across multiple computers (e.g., perception on GPU, control on real-time CPU, AI on separate processor), the decentralized architecture of ROS 2 is essential.

### Communication Protocols

**ROS 1:**
- TCPROS and UDPROS protocols
- Custom serialization formats
- Limited Quality of Service (QoS) options
- Difficult to integrate with external systems

**ROS 2:**
- DDS/RTPS-based communication
- Standardized serialization (ROS IDL)
- Rich QoS policies for different communication needs
- Easy integration with external systems using standard protocols

## Key Improvements for Humanoid Robotics

### 1. Quality of Service (QoS) Policies

One of the most significant improvements in ROS 2 is the introduction of Quality of Service policies, which allow fine-tuning of communication behavior:

```yaml
Reliability: BEST_EFFORT vs RELIABLE
Durability: VOLATILE vs TRANSIENT_LOCAL
History: KEEP_LAST vs KEEP_ALL
Depth: Buffer size for message queues
```

For humanoid robotics, this enables:
- **Critical control messages**: Use RELIABLE with small buffers for immediate delivery
- **Sensor data**: Use BEST_EFFORT with larger buffers to avoid blocking
- **Configuration data**: Use TRANSIENT_LOCAL to ensure late-joining nodes receive parameters

### 2. Real-Time Performance

ROS 2 includes several improvements for real-time performance:

- **Better thread safety**: Improved mutex handling and reduced lock contention
- **Deterministic behavior**: More predictable message delivery timing
- **Real-time scheduling support**: Better integration with real-time scheduling policies
- **Reduced latency**: More efficient message passing mechanisms

For humanoid robots that must maintain balance and respond to environmental changes within milliseconds, these improvements are crucial.

### 3. Security Features

ROS 2 introduces comprehensive security features:

- **Authentication**: Verify identity of nodes joining the system
- **Encryption**: Encrypt message contents for confidentiality
- **Access control**: Control which nodes can publish/subscribe to topics
- **Secure communication**: End-to-end security for distributed systems

Humanoid robots operating in human environments must ensure security to prevent unauthorized control or data access.

### 4. Lifecycle Management

ROS 2 provides built-in lifecycle management for nodes:

```
UNCONFIGURED → INACTIVE → ACTIVE → FINALIZED
     ↑                                    ↓
     └─────────── INACTIVE ←──────────────┘
```

This allows for:
- Graceful startup and shutdown sequences
- Configuration management
- Resource allocation and deallocation
- Error recovery and restart procedures

For complex humanoid systems with many interdependent components, lifecycle management is essential for safe operation.

## Practical Comparison: Humanoid Robot Example

Let's compare how a humanoid robot's walking control system would differ between ROS 1 and ROS 2:

### ROS 1 Walking Controller

```python
#!/usr/bin/env python3
# ROS 1 walking controller example
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class WalkingControllerROS1:
    def __init__(self):
        rospy.init_node('walking_controller')

        # Publishers for joint commands
        self.l_hip_pub = rospy.Publisher('/l_hip_position_controller/command', Float64, queue_size=1)
        self.r_hip_pub = rospy.Publisher('/r_hip_position_controller/command', Float64, queue_size=1)

        # Subscriber for sensor data
        rospy.Subscriber('/joint_states', JointState, self.joint_callback)

        self.rate = rospy.Rate(100)  # 100 Hz control loop

    def joint_callback(self, msg):
        # Process joint state data
        pass

    def run(self):
        while not rospy.is_shutdown():
            # Calculate walking control commands
            hip_pos = self.calculate_walking_pattern()

            # Publish commands
            self.l_hip_pub.publish(hip_pos['left'])
            self.r_hip_pub.publish(hip_pos['right'])

            self.rate.sleep()

if __name__ == '__main__':
    controller = WalkingControllerROS1()
    controller.run()
```

### ROS 2 Walking Controller

```python
#!/usr/bin/env python3
# ROS 2 walking controller example with QoS
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class WalkingControllerROS2(Node):
    def __init__(self):
        super().__init__('walking_controller')

        # QoS profile for critical control messages
        control_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # Publishers for joint commands
        self.l_hip_pub = self.create_publisher(Float64, '/l_hip_position_controller/command', control_qos)
        self.r_hip_pub = self.create_publisher(Float64, '/r_hip_position_controller/command', control_qos)

        # QoS profile for sensor data (higher buffer, best effort)
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscriber for sensor data
        self.sensor_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, sensor_qos)

        # Create timer for control loop
        self.timer = self.create_timer(0.01, self.control_loop)  # 100 Hz control loop

    def joint_callback(self, msg):
        # Process joint state data
        pass

    def control_loop(self):
        # Calculate walking control commands
        hip_pos = self.calculate_walking_pattern()

        # Publish commands
        self.l_hip_pub.publish(Float64(data=hip_pos['left']))
        self.r_hip_pub.publish(Float64(data=hip_pos['right']))

def main(args=None):
    rclpy.init(args=args)
    controller = WalkingControllerROS2()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Impact on Humanoid Robotics Development

### Advantages of ROS 2 for Humanoid Robots

1. **Reliability**: No single point of failure, better error handling
2. **Performance**: Better real-time capabilities, reduced latency
3. **Security**: Essential for robots operating in human environments
4. **Distribution**: Natural support for multi-computer architectures
5. **Maintenance**: Better tooling for debugging and monitoring complex systems

### Migration Considerations

When migrating from ROS 1 to ROS 2 for humanoid robotics:

1. **Communication patterns**: Update to use QoS profiles appropriately
2. **Threading model**: Adapt to ROS 2's threading model
3. **Package structure**: Update package.xml and CMakeLists.txt for ROS 2
4. **Tool usage**: Learn new tools like `ros2 topic`, `ros2 service`, etc.
5. **Testing**: Update tests to use ROS 2 testing frameworks

## Real-World Applications

Many humanoid robotics projects have successfully adopted ROS 2:

- **NAO/Hydrogen**: SoftBank Robotics' humanoid platform
- **Pepper**: Used in research and commercial applications
- **Atlas**: Boston Dynamics' humanoid (though not open source)
- **Various university research platforms**: Stanford, MIT, CMU humanoid projects

## Key Takeaways

- ROS 2's decentralized architecture eliminates single points of failure
- QoS policies enable fine-grained control over communication behavior
- Real-time improvements make ROS 2 suitable for safety-critical applications
- Security features protect against unauthorized access and control
- Lifecycle management provides structured node initialization and cleanup

## Next Steps

- Install ROS 2 Humble Hawksbill on your development system
- Create your first ROS 2 workspace for humanoid robotics
- Experiment with different QoS policies for various communication patterns
- Practice using ROS 2 command-line tools for debugging and monitoring

## References

- ROS 2 Migration Guide: https://docs.ros.org/en/humble/Releases/Migration-Guide.html
- Quality of Service in ROS 2: https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html
- ROS 2 Security: https://docs.ros.org/en/humble/Concepts/About-Security.html
- Real-Time Performance in ROS 2: https://docs.ros.org/en/humble/How-To-Guides/Real-Time-Programming.html