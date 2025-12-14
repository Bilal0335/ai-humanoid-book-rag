---
title: ROS 2 Nodes, Topics, Services, and Actions for Humanoid Robotics
sidebar_label: Nodes, Topics, Services, Actions
description: Understanding ROS 2 communication primitives for humanoid robot control and coordination
---

# ROS 2 Nodes, Topics, Services, and Actions for Humanoid Robotics

## Learning Objectives

- Understand the fundamental ROS 2 communication primitives: nodes, topics, services, and actions
- Implement communication patterns appropriate for humanoid robotics applications
- Design effective message structures for robot control and sensor data
- Apply Quality of Service (QoS) policies for different communication needs
- Create robust communication architectures for complex humanoid systems

## Introduction

Communication is the backbone of any robotic system, and for humanoid robots with dozens of sensors, actuators, and processing units, effective communication patterns are essential. ROS 2 provides four primary communication mechanisms that serve different purposes in humanoid robotics:

1. **Nodes**: Computational units that perform specific functions
2. **Topics**: Asynchronous, many-to-many communication for continuous data streams
3. **Services**: Synchronous, request-response communication for discrete operations
4. **Actions**: Goal-oriented communication for long-running tasks with feedback

Understanding when to use each mechanism and how to configure them appropriately is crucial for building reliable humanoid robotics systems.

## ROS 2 Nodes in Humanoid Robotics

### What Are Nodes?

Nodes are the fundamental computational units in ROS 2. Each node performs a specific function and communicates with other nodes through topics, services, and actions. In humanoid robotics, nodes might represent:

- Individual sensor drivers (IMU, cameras, LiDAR)
- Actuator controllers (joint position, velocity, effort controllers)
- Perception systems (object detection, SLAM, computer vision)
- Motion planning systems (path planning, trajectory generation)
- High-level AI reasoning (behavior selection, task planning)
- System monitoring and diagnostics

### Node Architecture for Humanoid Systems

```
┌─────────────────────────────────────────────────────────────────┐
│                    Humanoid Robot Node Graph                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐        │
│  │   IMU Node  │    │  Camera     │    │   LiDAR     │        │
│  │             │◄──►│   Node      │◄──►│   Node      │        │
│  └─────────────┘    └─────────────┘    └─────────────┘        │
│         │                   │                   │             │
│         ▼                   ▼                   ▼             │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │              Perception Processing Node               │   │
│  │         (Fuses sensor data to create world model)      │   │
│  └─────────────────────────────────────────────────────────┘   │
│                              │                                │
│                              ▼                                │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │              Motion Planning Node                     │   │
│  │      (Generates trajectories and action plans)        │   │
│  └─────────────────────────────────────────────────────────┘   │
│                              │                                │
│                              ▼                                │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │              Joint Control Node                       │   │
│  │     (Controls individual joint positions/velocities)  │   │
│  └─────────────────────────────────────────────────────────┘   │
│                              │                                │
│                              ▼                                │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │              Diagnostic Monitoring                    │   │
│  │        (Monitors system health and performance)       │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Creating Nodes for Humanoid Applications

#### Python Node Example: Joint State Publisher

```python
#!/usr/bin/env python3
"""
Joint State Publisher Node for Humanoid Robotics
Publishes joint states for all humanoid joints at 100Hz
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import time

class HumanoidJointStatePublisher(Node):
    def __init__(self):
        super().__init__('humanoid_joint_state_publisher')

        # Create publisher for joint states
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        # Define humanoid joint names (example for simplified humanoid)
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint', 'left_wrist_joint',
            'right_shoulder_joint', 'right_elbow_joint', 'right_wrist_joint',
            'neck_joint', 'waist_joint'
        ]

        # Initialize joint positions (starting position)
        self.joint_positions = [0.0] * len(self.joint_names)

        # Create timer to publish at 100Hz
        self.timer = self.create_timer(0.01, self.publish_joint_states)

        self.get_logger().info('Humanoid Joint State Publisher initialized')

    def publish_joint_states(self):
        """Publish joint state messages at regular intervals"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.name = self.joint_names
        msg.position = self.joint_positions

        # Simulate some joint movement (for demonstration)
        t = time.time()
        for i in range(len(self.joint_positions)):
            self.joint_positions[i] = 0.1 * math.sin(t + i * 0.1)

        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidJointStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### C++ Node Example: Balance Controller

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class HumanoidBalanceController : public rclcpp::Node
{
public:
    HumanoidBalanceController() : Node("humanoid_balance_controller")
    {
        // Create subscription to IMU data
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&HumanoidBalanceController::imu_callback, this, std::placeholders::_1));

        // Create publisher for corrective joint commands
        correction_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
            "/balance_corrections", 10);

        // Create timer for control loop (200Hz for balance control)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&HumanoidBalanceController::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Balance controller initialized");
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Store latest IMU data for control calculations
        latest_imu_ = *msg;
        has_new_imu_data_ = true;
    }

    void control_loop()
    {
        if (!has_new_imu_data_) return;

        // Calculate balance corrections based on IMU data
        auto quaternion = tf2::Quaternion(
            latest_imu_.orientation.x,
            latest_imu_.orientation.y,
            latest_imu_.orientation.z,
            latest_imu_.orientation.w
        );

        // Convert to Euler angles to determine tilt
        tf2::Matrix3x3 matrix(quaternion);
        double roll, pitch, yaw;
        matrix.getRPY(roll, pitch, yaw);

        // Calculate corrective commands based on tilt
        control_msgs::msg::JointJog correction_msg;
        correction_msg.header.stamp = this->get_clock()->now();

        // Example: Adjust hip joints to counteract forward/backward tilt
        if (abs(pitch) > 0.1) {  // Threshold for balance correction
            correction_msg.joint_names = {"left_hip_pitch", "right_hip_pitch"};
            correction_msg.displacements = {-pitch * 0.5, -pitch * 0.5};  // Proportional correction
            correction_msg.velocities = {0.0, 0.0};
        }

        correction_pub_->publish(correction_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr correction_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    sensor_msgs::msg::Imu latest_imu_;
    bool has_new_imu_data_ = false;
};

int main(int argc, char * argv[])
{
    rclpy::init(argc, argv);
    rclpy::spin(std::make_shared<HumanoidBalanceController>());
    rclpy::shutdown();
    return 0;
}
```

## Topics: Continuous Data Streams

### When to Use Topics

Topics are ideal for continuous data streams where:

- Multiple nodes need to consume the same data
- Data is published at regular intervals
- Consumers don't need to know about each other
- Some message loss is acceptable (BEST_EFFORT) or not (RELIABLE)

For humanoid robotics, topics are perfect for:

- Sensor data streams (camera images, LiDAR scans, IMU data)
- Joint states and motor feedback
- Robot transforms (TF)
- Perception results (detected objects, landmarks)
- System diagnostics and performance metrics

### Topic Design for Humanoid Sensors

```python
#!/usr/bin/env python3
"""
Humanoid Sensor Data Aggregator
Demonstrates proper topic usage for multiple sensor streams
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan, Image, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class HumanoidSensorAggregator(Node):
    def __init__(self):
        super().__init__('humanoid_sensor_aggregator')

        # Different QoS profiles for different sensor types
        # High-priority sensors (IMU) - RELIABLE, small buffer
        imu_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # Medium-priority sensors (LiDAR) - BEST_EFFORT, larger buffer
        lidar_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # Low-priority sensors (Cameras) - BEST_EFFORT, large buffer
        camera_qos = QoSProfile(
            depth=15,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # Create subscriptions with appropriate QoS
        self.imu_sub = self.create_subscription(Imu, '/imu/data_raw',
                                               self.imu_callback, imu_qos)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan',
                                                 self.lidar_callback, lidar_qos)
        self.camera_sub = self.create_subscription(Image, '/camera/rgb/image_raw',
                                                  self.camera_callback, camera_qos)
        self.joint_sub = self.create_subscription(JointState, '/joint_states',
                                                self.joint_callback, imu_qos)

        # Create publishers for processed data
        self.fused_sensor_pub = self.create_publisher(Twist, '/sensor_fusion/velocity_estimate', 10)
        self.safety_status_pub = self.create_publisher(Bool, '/safety/system_ok', 10)

        # Store latest sensor data
        self.latest_imu = None
        self.latest_lidar = None
        self.latest_camera = None
        self.latest_joints = None

        self.get_logger().info('Humanoid Sensor Aggregator initialized')

    def imu_callback(self, msg):
        self.latest_imu = msg
        self.process_sensor_fusion()

    def lidar_callback(self, msg):
        self.latest_lidar = msg
        self.process_sensor_fusion()

    def camera_callback(self, msg):
        self.latest_camera = msg
        self.process_sensor_fusion()

    def joint_callback(self, msg):
        self.latest_joints = msg
        self.process_sensor_fusion()

    def process_sensor_fusion(self):
        """Process sensor data and publish fused estimates"""
        if all([self.latest_imu, self.latest_lidar]):
            # Simple sensor fusion example
            fused_msg = Twist()

            # Estimate velocity from IMU acceleration
            fused_msg.linear.x = self.latest_imu.linear_acceleration.x * 0.1
            fused_msg.angular.z = self.latest_imu.angular_velocity.z

            self.fused_sensor_pub.publish(fused_msg)

            # Check safety conditions
            safe = self.check_safety_conditions()
            safety_msg = Bool()
            safety_msg.data = safe
            self.safety_status_pub.publish(safety_msg)

    def check_safety_conditions(self):
        """Check if robot is in safe operating conditions"""
        if self.latest_imu is None:
            return False

        # Check if robot is tilting too much
        orientation = self.latest_imu.orientation
        # Simple check: if angle from vertical is too large, robot is unsafe
        tilt_threshold = 0.5  # radians
        tilt_magnitude = abs(orientation.z) + abs(orientation.w)

        return tilt_magnitude > tilt_threshold

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidSensorAggregator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Quality of Service (QoS) for Humanoid Applications

Different QoS profiles should be used based on the criticality of the data:

| Data Type | Reliability | History | Depth | Durability | Use Case |
|-----------|-------------|---------|-------|------------|----------|
| Joint commands | RELIABLE | KEEP_LAST | 1 | VOLATILE | Critical control data |
| IMU data | RELIABLE | KEEP_LAST | 5 | VOLATILE | Balance control |
| LiDAR scans | BEST_EFFORT | KEEP_LAST | 10 | VOLATILE | Navigation, obstacle detection |
| Camera images | BEST_EFFORT | KEEP_LAST | 15 | VOLATILE | Perception (occasional loss OK) |
| Joint states | RELIABLE | KEEP_LAST | 10 | VOLATILE | Feedback for control |
| TF transforms | RELIABLE | KEEP_ALL | 100 | TRANSIENT_LOCAL | Coordinate system management |

## Services: Discrete Operations

### When to Use Services

Services are ideal for discrete operations where:

- Request-response pattern is needed
- Operation should complete before proceeding
- Result is critical for next step
- Operation is not time-critical (can block)

For humanoid robotics, services work well for:

- Changing robot parameters or configuration
- Requesting specific computations (IK, path planning)
- Saving/restoring robot state
- Activating/deactivating safety systems
- Requesting calibration or homing procedures

### Service Design for Humanoid Applications

```python
#!/usr/bin/env python3
"""
Humanoid Configuration Service
Provides service calls for humanoid-specific configuration operations
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_srvs.srv import SetBool
from sensor_msgs.srv import SetCameraInfo
from humanoid_msgs.srv import SetWalkingPattern, GetRobotState

class HumanoidConfigService(Node):
    def __init__(self):
        super().__init__('humanoid_config_service')

        # Service to activate/deactivate walking
        self.walk_service = self.create_service(
            SetBool,
            '/humanoid/control/walk_enabled',
            self.handle_walk_enable_request
        )

        # Service to set walking pattern
        self.pattern_service = self.create_service(
            SetWalkingPattern,
            '/humanoid/control/set_walking_pattern',
            self.handle_pattern_request
        )

        # Service to get current robot state
        self.state_service = self.create_service(
            GetRobotState,
            '/humanoid/status/get_robot_state',
            self.handle_state_request
        )

        # Service to home all joints
        self.home_service = self.create_service(
            SetBool,
            '/humanoid/control/home_joints',
            self.handle_home_request
        )

        # Internal state
        self.walking_enabled = False
        self.current_pattern = "default"
        self.homing_active = False

        self.get_logger().info('Humanoid Configuration Service initialized')

    def handle_walk_enable_request(self, request, response):
        """Handle request to enable/disable walking"""
        try:
            self.walking_enabled = request.data
            response.success = True
            response.message = f"Walking {'enabled' if self.walking_enabled else 'disabled'}"

            self.get_logger().info(response.message)
            return response
        except Exception as e:
            response.success = False
            response.message = f"Failed to change walk state: {str(e)}"
            return response

    def handle_pattern_request(self, request, response):
        """Handle request to set walking pattern"""
        try:
            if request.pattern in ["walk", "trot", "crawl", "balance"]:
                self.current_pattern = request.pattern
                response.success = True
                response.message = f"Set walking pattern to {request.pattern}"

                self.get_logger().info(response.message)
                return response
            else:
                response.success = False
                response.message = f"Invalid walking pattern: {request.pattern}. Valid: walk, trot, crawl, balance"
                return response
        except Exception as e:
            response.success = False
            response.message = f"Failed to set pattern: {str(e)}"
            return response

    def handle_state_request(self, request, response):
        """Handle request to get robot state"""
        try:
            # Fill response with current robot state
            response.state.walking_enabled = self.walking_enabled
            response.state.current_pattern = self.current_pattern
            response.state.homing_active = self.homing_active
            response.state.timestamp = self.get_clock().now().to_msg()

            response.success = True
            response.message = "State retrieved successfully"

            return response
        except Exception as e:
            response.success = False
            response.message = f"Failed to get state: {str(e)}"
            return response

    def handle_home_request(self, request, response):
        """Handle request to home all joints"""
        try:
            if request.data:  # If request is to activate homing
                # This would trigger a more complex homing sequence
                # In practice, this might be better implemented as an Action
                self.homing_active = True
                response.success = True
                response.message = "Started homing sequence"

                # Simulate homing process completion
                import time
                time.sleep(2)  # Simulate homing time
                self.homing_active = False

                self.get_logger().info("Homing sequence completed")
            else:
                response.success = False
                response.message = "Homing deactivation not supported"

            return response
        except Exception as e:
            response.success = False
            response.message = f"Failed to home joints: {str(e)}"
            return response

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidConfigService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Actions: Long-Running Tasks with Feedback

### When to Use Actions

Actions are perfect for long-running tasks where:

- Goal-oriented behavior is needed
- Feedback during execution is important
- Cancelation capabilities are required
- Result is complex and may take time to generate

For humanoid robotics, actions are essential for:

- Navigation tasks (going to specific locations)
- Manipulation tasks (grasping, placing objects)
- Walking and locomotion
- Complex perception tasks (mapping, object recognition)
- Calibration procedures

### Action Design for Humanoid Applications

```python
#!/usr/bin/env python3
"""
Humanoid Walking Action Server
Implements a walking action for humanoid robots with feedback and result reporting
"""
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from humanoid_msgs.action import WalkToPose
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
import time
import math

class HumanoidWalkActionServer(Node):
    def __init__(self):
        super().__init__('humanoid_walk_action_server')

        # Create action server
        self._action_server = ActionServer(
            self,
            WalkToPose,
            'humanoid/walk_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Publishers for joint commands during walking
        self.left_hip_pub = self.create_publisher(Float64, '/left_hip_controller/command', 10)
        self.right_hip_pub = self.create_publisher(Float64, '/right_hip_controller/command', 10)
        self.left_knee_pub = self.create_publisher(Float64, '/left_knee_controller/command', 10)
        self.right_knee_pub = self.create_publisher(Float64, '/right_knee_controller/command', 10)

        self.get_logger().info('Humanoid Walk Action Server initialized')

    def goal_callback(self, goal_request):
        """Accept or reject a goal"""
        # Check if goal is valid (within reasonable bounds)
        if abs(goal_request.target_pose.position.x) > 100.0:  # 100m max range
            self.get_logger().warn('Goal position too far away, rejecting')
            return GoalResponse.REJECT

        self.get_logger().info(f'Accepted goal: ({goal_request.target_pose.position.x}, {goal_request.target_pose.position.y})')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a cancel request"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the walking goal"""
        self.get_logger().info('Executing walking goal')

        feedback_msg = WalkToPose.Feedback()
        result_msg = WalkToPose.Result()

        target_x = goal_handle.request.target_pose.position.x
        target_y = goal_handle.request.target_pose.position.y

        # Simple walking simulation - in reality this would involve complex gait planning
        current_x = 0.0  # Starting position
        current_y = 0.0

        # Calculate distance to target
        total_distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        step_size = 0.1  # 10cm per step

        steps_needed = int(total_distance / step_size)

        for i in range(steps_needed):
            if goal_handle.is_cancel_requested:
                result_msg.success = False
                result_msg.message = 'Goal canceled'
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return result_msg

            # Simulate taking a step
            progress_fraction = float(i) / steps_needed
            current_x = target_x * progress_fraction
            current_y = target_y * progress_fraction

            # Send joint commands for walking gait
            # This is a simplified example - real walking would be much more complex
            left_hip_angle = 0.1 * math.sin(i * 0.5)
            right_hip_angle = 0.1 * math.sin(i * 0.5 + math.pi)

            self.left_hip_pub.publish(Float64(data=left_hip_angle))
            self.right_hip_pub.publish(Float64(data=right_hip_angle))

            # Update feedback
            feedback_msg.current_pose.position.x = current_x
            feedback_msg.current_pose.position.y = current_y
            feedback_msg.distance_remaining = total_distance - (progress_fraction * total_distance)
            feedback_msg.progress_percentage = progress_fraction * 100.0

            goal_handle.publish_feedback(feedback_msg)

            # Sleep to simulate walking time
            time.sleep(0.2)

        # Goal completed
        result_msg.success = True
        result_msg.message = f'Reached target position ({target_x}, {target_y})'
        goal_handle.succeed()

        self.get_logger().info(f'Goal succeeded: {result_msg.message}')
        return result_msg

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidWalkActionServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Communication Patterns in Humanoid Robotics

### 1. Sensor Fusion Pattern

Multiple sensors feed into a central fusion node:

```
Multiple Sensor Nodes → Sensor Fusion Node → Perception/Planning Nodes
```

This pattern is critical for humanoid robots that need to integrate data from IMU, cameras, LiDAR, joint encoders, and force/torque sensors.

### 2. Behavior Tree Integration Pattern

Actions coordinate with behavior trees for complex task execution:

```
High-Level Planner → Behavior Tree → Action Clients → Robot Control Nodes
```

### 3. Safety Monitor Pattern

A safety monitor watches all system activity and can intervene:

```
All Nodes → Safety Monitor → Emergency Stop/Intervention
```

### 4. Coordination Pattern

A coordinator manages multiple subsystems:

```
Coordinator ↔ Perception ↔ Navigation ↔ Manipulation ↔ Control
```

## Testing Communication Patterns

### Unit Testing for Communication

```python
#!/usr/bin/env python3
# test_communication.py
import unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

class TestHumanoidCommunication(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('test_humanoid_communication')

        # Create test publishers and subscribers
        self.test_publisher = self.node.create_publisher(JointState, '/test/joint_states', 10)
        self.received_messages = []

        def msg_callback(msg):
            self.received_messages.append(msg)

        self.test_subscriber = self.node.create_subscription(
            JointState, '/test/joint_states', msg_callback, 10
        )

        # Create test service client
        self.service_client = self.node.create_client(SetBool, '/test_service')

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_topic_publish_subscribe(self):
        """Test basic topic publish/subscribe functionality"""
        # Create a test message
        test_msg = JointState()
        test_msg.name = ['test_joint']
        test_msg.position = [1.0]

        # Publish message
        self.test_publisher.publish(test_msg)

        # Spin briefly to allow message to be processed
        self.executor.spin_once(timeout_sec=0.1)

        # Check that message was received
        self.assertEqual(len(self.received_messages), 1)
        self.assertEqual(self.received_messages[0].position[0], 1.0)

    def test_service_call(self):
        """Test service call functionality"""
        # Skip if service is not available (would normally be provided by another node)
        if not self.service_client.service_is_ready():
            self.skipTest("Service not available for testing")

if __name__ == '__main__':
    unittest.main()
```

### Integration Testing

```bash
# Test communication patterns using command line tools
# Check if all expected topics are available
ros2 topic list | grep -E "joint_states|imu|scan|tf"

# Echo joint states to verify they're being published
ros2 topic echo /joint_states --field position

# Call a service to verify it's working
ros2 service call /humanoid/control/walk_enabled std_srvs/SetBool '{data: true}'

# Check action server status
ros2 action list | grep humanoid
```

## Performance Optimization

### 1. Message Efficiency

For humanoid robotics with many joints and sensors:

- Use appropriate message types (Float64MultiArray vs individual Float64)
- Optimize message frequency (100Hz for control, 30Hz for perception)
- Use appropriate QoS profiles for different data types

### 2. Network Optimization

- Use intra-process communication when nodes run on the same machine
- Configure DDS for low-latency communication between critical nodes
- Consider different middleware implementations based on performance needs

### 3. Memory Management

- Use appropriate queue depths to balance responsiveness and memory usage
- Implement message buffering strategies for high-frequency sensors
- Monitor memory usage during long-running operations

## Best Practices for Humanoid Communication

### 1. Naming Conventions

Use consistent naming for humanoid robotics:

```
# Joint commands
/left_leg_controller/position_commands
/right_arm_controller/effort_commands

# Sensor data
/body_imu/data
/head_camera/image_raw
/foot_pressure_sensors/lf_foot

# Actions
/navigation/walk_to_pose
/manipulation/grasp_object
/diagnostic/run_self_test
```

### 2. Error Handling

Always implement proper error handling:

```python
def handle_sensor_data(self, msg):
    if msg is None:
        self.get_logger().warn('Received null sensor message')
        return

    if len(msg.position) != len(self.expected_joints):
        self.get_logger().error(f'Joint count mismatch: expected {len(self.expected_joints)}, got {len(msg.position)}')
        return

    # Process valid message
    self.process_valid_sensor_data(msg)
```

### 3. Safety Considerations

For humanoid robots operating near humans:

- Implement emergency stop topics that all nodes monitor
- Use appropriate timeouts for all communication
- Validate all incoming messages before processing
- Implement graceful degradation when communication fails

## Next Steps

1. Implement the communication patterns in your humanoid simulation
2. Test QoS configurations for your specific use case
3. Create launch files to start your communication architecture
4. Implement diagnostic nodes to monitor communication health
5. Set up logging for debugging communication issues

## References

- ROS 2 Topics: https://docs.ros.org/en/humble/Concepts/About-Topics.html
- ROS 2 Services: https://docs.ros.org/en/humble/Concepts/About-Services.html
- ROS 2 Actions: https://docs.ros.org/en/humble/Concepts/About-Actions.html
- Quality of Service: https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html
- ROS 2 Node Design: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Node.html