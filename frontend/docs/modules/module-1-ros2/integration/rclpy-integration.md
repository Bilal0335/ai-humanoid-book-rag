---
title: Using rclpy to Connect Python Agents to ROS 2
sidebar_label: rclpy Integration
description: Connecting Python-based AI agents to ROS 2 systems for humanoid robotics applications
---

# Using rclpy to Connect Python Agents to ROS 2

## Learning Objectives

- Understand the rclpy client library and its role in ROS 2 Python development
- Implement Python agents that can communicate with ROS 2 systems using rclpy
- Design AI-to-ROS 2 control pipelines for humanoid robotics applications
- Create command pipelines connecting LLMs to ROS 2 controllers
- Validate Python agent integration with ROS 2 communication patterns

## Introduction

The integration of Python-based AI agents with ROS 2 systems is a critical capability for modern humanoid robotics. Python's rich ecosystem of AI and machine learning libraries combined with ROS 2's robust communication infrastructure enables sophisticated robotic behaviors. The `rclpy` client library provides the essential bridge between Python AI agents and the ROS 2 middleware, allowing for seamless integration of high-level reasoning with low-level robot control.

For humanoid robotics applications, this integration is particularly important because:

1. **AI Libraries**: Python has extensive libraries for machine learning, natural language processing, and computer vision
2. **Rapid Prototyping**: Python enables quick iteration and testing of AI algorithms
3. **Integration**: Python agents can coordinate multiple ROS 2 subsystems effectively
4. **Flexibility**: Easy to extend and modify AI behaviors without rebuilding ROS 2 nodes

## Understanding rclpy Architecture

### rclpy vs. rclcpp

While ROS 2 provides client libraries for multiple languages, `rclpy` specifically enables Python integration:

- **rclpy**: Python client library, ideal for AI agents and rapid prototyping
- **rclcpp**: C++ client library, ideal for performance-critical real-time control
- **Both**: Share the same underlying ROS middleware (RMW) and DDS implementations

### Core Components of rclpy

The rclpy library provides several key components for Python-ROS 2 integration:

```
┌─────────────────────────────────────────────────────────────────┐
│                        rclpy Architecture                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────┐ │
│  │   Python Agent  │    │    rclpy        │    │  ROS 2      │ │
│  │   (AI Logic)    │◄──►│  Client Lib     │◄──►│  Middleware │ │
│  └─────────────────┘    │  (rclpy)        │    │  (DDS/RMW)  │ │
│                         └─────────────────┘    └─────────────┘ │
│                               │                        │        │
│  ┌─────────────────┐    ┌─────────────────┐         │        │
│  │  Node Interface │    │  Async Handler  │         │        │
│  │ (create_publisher│    │ (executors,    │         │        │
│  │  create_service,│    │  futures, etc.) │         │        │
│  │  etc.)          │    │                 │         │        │
│  └─────────────────┘    └─────────────────┘         │        │
│         │                       │                    │        │
│         └───────────────────────┼────────────────────┘        │
│                                 ▼                             │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │                    ROS 2 Graph                        │   │
│  │   (other nodes, topics, services, actions)            │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Key Classes and Functions

1. **Node**: The fundamental unit of computation in ROS 2
2. **Publisher**: Sends messages to topics
3. **Subscriber**: Receives messages from topics
4. **Client**: Calls services
5. **Service**: Provides service callbacks
6. **ActionClient**: Sends action goals
7. **ActionServer**: Executes action goals
8. **Timer**: Executes callbacks at regular intervals
9. **Parameter**: Manages node parameters

## Setting Up rclpy Environment

### 1. Installation and Dependencies

First, ensure your Python environment is properly set up for rclpy:

```bash
# Make sure ROS 2 is sourced
source /opt/ros/humble/setup.bash

# Create a virtual environment for your Python agent
python3 -m venv humanoid_ai_env
source humanoid_ai_env/bin/activate

# Install rclpy (usually comes with ROS 2 installation)
pip install rclpy

# Install additional AI libraries for humanoid applications
pip install numpy scipy matplotlib
pip install openai anthropic  # For LLM integration
pip install opencv-python   # For computer vision
pip install torch torchvision  # For neural networks
pip install pyquaternion    # For rotation mathematics
```

### 2. Basic rclpy Node Structure

Here's the basic structure for a Python agent node:

```python
#!/usr/bin/env python3
"""
Basic Python Agent Node Template
Template for connecting AI agents to ROS 2 using rclpy
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

class PythonAIAgent(Node):
    def __init__(self):
        # Initialize the node with a name
        super().__init__('python_ai_agent')

        # Create publishers for sending commands to robot
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_cmd_pub = self.create_publisher(Float64, '/joint_commands', 10)

        # Create subscribers for receiving sensor data
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Create services for interaction
        self.control_service = self.create_service(
            SetBool,
            '/ai_agent/control_enabled',
            self.control_service_callback
        )

        # Create timers for periodic AI processing
        self.ai_timer = self.create_timer(0.1, self.ai_processing_callback)  # 10Hz

        # Internal state
        self.control_enabled = True
        self.latest_joint_state = None
        self.latest_imu_data = None

        self.get_logger().info('Python AI Agent initialized')

    def joint_state_callback(self, msg):
        """Handle incoming joint state messages"""
        self.latest_joint_state = msg
        self.get_logger().debug(f'Received joint state with {len(msg.name)} joints')

    def imu_callback(self, msg):
        """Handle incoming IMU data"""
        self.latest_imu_data = msg
        # Process balance information from IMU
        self.process_balance_data(msg)

    def ai_processing_callback(self):
        """Main AI processing loop"""
        if not self.control_enabled:
            return

        # Perform AI reasoning based on current state
        ai_decision = self.reason_about_state()

        # Execute decision (send commands to robot)
        if ai_decision:
            self.execute_decision(ai_decision)

    def control_service_callback(self, request, response):
        """Handle service calls to enable/disable AI control"""
        self.control_enabled = request.data
        response.success = True
        response.message = f'Control {"enabled" if self.control_enabled else "disabled"}'
        self.get_logger().info(response.message)
        return response

    def reason_about_state(self):
        """Perform AI reasoning based on current robot state"""
        # Placeholder for AI reasoning logic
        # In practice, this would call LLMs, neural networks, or other AI systems
        if self.latest_joint_state and self.latest_imu_data:
            # Example: Check if robot is upright based on IMU data
            orientation = self.latest_imu_data.orientation
            # Simple balance check
            if abs(orientation.z) > 0.7:  # Robot is roughly upright
                return {"action": "maintain_position", "params": {}}
            else:
                return {"action": "attempt_balance", "params": {}}
        return None

    def execute_decision(self, decision):
        """Execute AI decision by sending appropriate ROS 2 commands"""
        if decision["action"] == "maintain_position":
            # Send zero velocity command
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
        elif decision["action"] == "attempt_balance":
            # Send balance correction command
            # This would involve more complex joint control in practice
            pass

def main(args=None):
    rclpy.init(args=args)
    node = PythonAIAgent()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced rclpy Patterns for Humanoid AI

### 1. Async/Await Pattern for Non-Blocking Operations

For humanoid applications requiring complex AI processing without blocking the ROS 2 executor:

```python
import asyncio
import aiohttp
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import threading
import concurrent.futures

class AsyncPythonAIAgent(Node):
    def __init__(self):
        super().__init__('async_python_ai_agent')

        # Publisher for AI results
        self.result_pub = self.create_publisher(String, '/ai_agent/results', 10)

        # Subscription for commands
        self.command_sub = self.create_subscription(
            String, '/ai_agent/commands', self.command_callback, 10
        )

        # Create executor for async operations
        self.executor = concurrent.futures.ThreadPoolExecutor(max_workers=4)

        # Create timer for periodic async tasks
        self.async_timer = self.create_timer(1.0, self.periodic_async_task)

        self.get_logger().info('Async Python AI Agent initialized')

    def command_callback(self, msg):
        """Handle incoming commands asynchronously"""
        # Submit AI processing to thread pool to avoid blocking
        future = self.executor.submit(self.process_command_async, msg.data)
        future.add_done_callback(self.on_ai_processing_complete)

    def process_command_async(self, command):
        """Process command in separate thread (non-blocking)"""
        import time
        time.sleep(0.5)  # Simulate AI processing time

        # In real implementation, this would call LLMs or ML models
        result = f"Processed: {command} at {time.time()}"
        return result

    def on_ai_processing_complete(self, future):
        """Callback when async AI processing is complete"""
        try:
            result = future.result()

            # Publish result back to ROS 2
            result_msg = String()
            result_msg.data = result
            self.result_pub.publish(result_msg)

            self.get_logger().info(f'Published AI result: {result}')
        except Exception as e:
            self.get_logger().error(f'AI processing failed: {str(e)}')

    def periodic_async_task(self):
        """Periodic task that doesn't block the main thread"""
        # This could trigger periodic AI model updates, data analysis, etc.
        self.get_logger().info('Executing periodic async task')

def main(args=None):
    rclpy.init(args=args)
    node = AsyncPythonAIAgent()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.executor.shutdown(wait=True)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Parameter Management for AI Configuration

Managing AI model parameters and configuration through ROS 2 parameters:

```python
#!/usr/bin/env python3
"""
Parameter Management for AI Agents
Using ROS 2 parameters to configure AI behavior
"""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64
import numpy as np

class ParameterizedAIAgent(Node):
    def __init__(self):
        super().__init__('parameterized_ai_agent')

        # Declare parameters with default values and descriptions
        self.declare_parameter('ai.model_temperature', 0.7,
                              ParameterDescriptor(description='Temperature for AI model sampling'))
        self.declare_parameter('ai.max_planning_steps', 50,
                              ParameterDescriptor(description='Maximum steps for AI planning'))
        self.declare_parameter('ai.confidence_threshold', 0.8,
                              ParameterDescriptor(description='Minimum confidence for AI actions'))
        self.declare_parameter('ai.learning_rate', 0.001,
                              ParameterDescriptor(description='Learning rate for adaptive AI'))

        # Publisher for AI commands
        self.command_pub = self.create_publisher(Float64, '/ai_commands', 10)

        # Timer for periodic AI processing
        self.ai_timer = self.create_timer(0.1, self.ai_processing_loop)

        # Subscribe to parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info('Parameterized AI Agent initialized')

    def parameter_callback(self, params):
        """Handle parameter changes"""
        for param in params:
            self.get_logger().info(f'Parameter {param.name} changed to {param.value}')

        return SetParametersResult(successful=True)

    def ai_processing_loop(self):
        """Main AI processing with parameterized behavior"""
        # Get current parameter values
        temperature = self.get_parameter('ai.model_temperature').value
        max_steps = self.get_parameter('ai.max_planning_steps').value
        confidence_threshold = self.get_parameter('ai.confidence_threshold').value
        learning_rate = self.get_parameter('ai.learning_rate').value

        # Use parameters in AI decision making
        ai_decision = self.make_decision_with_params(temperature, max_steps, confidence_threshold)

        if ai_decision and ai_decision['confidence'] >= confidence_threshold:
            # Publish command if confidence is sufficient
            cmd_msg = Float64()
            cmd_msg.data = ai_decision['command']
            self.command_pub.publish(cmd_msg)

    def make_decision_with_params(self, temperature, max_steps, confidence_threshold):
        """Make AI decision using current parameters"""
        # Example AI decision making with parameter influence
        # In practice, this would call actual AI models with these parameters

        # Simulate decision making influenced by parameters
        command = np.random.normal(scale=temperature)  # Temperature affects randomness
        confidence = np.random.uniform(0.5, 1.0)  # Random confidence for example

        return {
            'command': command,
            'confidence': confidence,
            'steps_evaluated': min(max_steps, 10)  # Example of parameter usage
        }

def main(args=None):
    rclpy.init(args=args)
    node = ParameterizedAIAgent()

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

## AI-to-ROS 2 Control Pipelines

### 1. LLM Integration Pattern

Connecting Large Language Models to ROS 2 control systems:

```python
#!/usr/bin/env python3
"""
LLM-to-ROS 2 Control Pipeline
Integrating LLMs with ROS 2 for natural language command processing
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from humanoid_msgs.msg import HumanoidCommand
from humanoid_msgs.srv import ExecuteAction
import openai
import json
import re

class LLMControlPipeline(Node):
    def __init__(self):
        super().__init__('llm_control_pipeline')

        # Publisher for robot commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.humanoid_cmd_pub = self.create_publisher(HumanoidCommand, '/humanoid/commands', 10)

        # Service client for action execution
        self.action_client = self.create_client(ExecuteAction, '/execute_humanoid_action')

        # Subscription for natural language commands
        self.nl_command_sub = self.create_subscription(
            String, '/natural_language_commands', self.nl_command_callback, 10
        )

        # Initialize OpenAI client
        try:
            self.openai_client = openai.OpenAI(api_key=self.get_parameter('openai.api_key').value)
        except:
            self.get_logger().warn('OpenAI API key not configured, using mock responses')
            self.openai_client = None

        # Define action mapping
        self.action_mapping = {
            'move_forward': self.execute_move_forward,
            'turn_left': self.execute_turn_left,
            'turn_right': self.execute_turn_right,
            'stop': self.execute_stop,
            'raise_arm': self.execute_raise_arm,
            'lower_arm': self.execute_lower_arm,
            'walk_to': self.execute_walk_to
        }

        self.get_logger().info('LLM Control Pipeline initialized')

    def nl_command_callback(self, msg):
        """Process natural language command using LLM"""
        try:
            # Parse the natural language command
            parsed_command = self.parse_natural_language_command(msg.data)

            if parsed_command:
                # Execute the parsed command
                self.execute_parsed_command(parsed_command)

        except Exception as e:
            self.get_logger().error(f'Error processing natural language command: {str(e)}')

    def parse_natural_language_command(self, command_text):
        """Use LLM to parse natural language into structured commands"""
        if self.openai_client:
            # Use OpenAI to parse the command
            prompt = f"""
            You are a command parser for a humanoid robot. Convert the following natural language command into a structured action:

            Command: "{command_text}"

            Respond in JSON format with:
            {{
                "action": "action_type",
                "parameters": {{"param1": "value1", "param2": "value2"}},
                "confidence": 0.0-1.0
            }}

            Valid actions: move_forward, turn_left, turn_right, stop, raise_arm, lower_arm, walk_to
            """

            try:
                response = self.openai_client.chat.completions.create(
                    model="gpt-3.5-turbo",
                    messages=[{"role": "user", "content": prompt}],
                    temperature=0.1
                )

                # Extract JSON from response
                response_text = response.choices[0].message.content.strip()

                # Clean up response to extract JSON
                json_match = re.search(r'\{.*\}', response_text, re.DOTALL)
                if json_match:
                    parsed_json = json.loads(json_match.group())
                    return parsed_json
            except Exception as e:
                self.get_logger().error(f'LLM parsing failed: {str(e)}')

        # Mock response if LLM is not available
        self.get_logger().warn(f'Using mock parser for command: {command_text}')
        return self.mock_parse_command(command_text)

    def mock_parse_command(self, command_text):
        """Mock command parsing for testing without LLM"""
        command_text_lower = command_text.lower()

        if 'forward' in command_text_lower or 'ahead' in command_text_lower:
            return {"action": "move_forward", "parameters": {"distance": 1.0}, "confidence": 0.9}
        elif 'left' in command_text_lower:
            return {"action": "turn_left", "parameters": {"angle": 90.0}, "confidence": 0.85}
        elif 'right' in command_text_lower:
            return {"action": "turn_right", "parameters": {"angle": 90.0}, "confidence": 0.85}
        elif 'stop' in command_text_lower:
            return {"action": "stop", "parameters": {}, "confidence": 0.95}
        elif 'raise' in command_text_lower and 'arm' in command_text_lower:
            return {"action": "raise_arm", "parameters": {"arm": "both"}, "confidence": 0.8}
        elif 'walk to' in command_text_lower:
            # Extract destination from command
            destination = "location"
            if 'kitchen' in command_text_lower:
                destination = 'kitchen'
            elif 'living room' in command_text_lower:
                destination = 'living_room'
            elif 'bedroom' in command_text_lower:
                destination = 'bedroom'

            return {"action": "walk_to", "parameters": {"destination": destination}, "confidence": 0.85}
        else:
            return {"action": "stop", "parameters": {}, "confidence": 0.5}

    def execute_parsed_command(self, parsed_command):
        """Execute the parsed command using appropriate method"""
        action = parsed_command.get('action')
        parameters = parsed_command.get('parameters', {})
        confidence = parsed_command.get('confidence', 0.0)

        if confidence < 0.7:  # Low confidence, ask for clarification
            self.get_logger().warn(f'Low confidence command: {parsed_command}')
            return

        if action in self.action_mapping:
            self.get_logger().info(f'Executing action: {action} with parameters: {parameters}')
            self.action_mapping[action](parameters)
        else:
            self.get_logger().error(f'Unknown action: {action}')

    # Action execution methods
    def execute_move_forward(self, parameters):
        """Execute move forward action"""
        twist_msg = Twist()
        twist_msg.linear.x = parameters.get('distance', 1.0)
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)

    def execute_turn_left(self, parameters):
        """Execute turn left action"""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = parameters.get('angle', 90.0) * 3.14159 / 180.0  # Convert to radians
        self.cmd_vel_pub.publish(twist_msg)

    def execute_turn_right(self, parameters):
        """Execute turn right action"""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = -parameters.get('angle', 90.0) * 3.14159 / 180.0  # Convert to radians
        self.cmd_vel_pub.publish(twist_msg)

    def execute_stop(self, parameters):
        """Execute stop action"""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)

    def execute_raise_arm(self, parameters):
        """Execute raise arm action"""
        cmd_msg = HumanoidCommand()
        cmd_msg.command_type = 'joint_position'
        cmd_msg.target_joints = ['left_shoulder_pitch', 'right_shoulder_pitch']
        cmd_msg.target_positions = [0.5, 0.5]  # Raise arms
        self.humanoid_cmd_pub.publish(cmd_msg)

    def execute_lower_arm(self, parameters):
        """Execute lower arm action"""
        cmd_msg = HumanoidCommand()
        cmd_msg.command_type = 'joint_position'
        cmd_msg.target_joints = ['left_shoulder_pitch', 'right_shoulder_pitch']
        cmd_msg.target_positions = [0.0, 0.0]  # Lower arms
        self.humanoid_cmd_pub.publish(cmd_msg)

    def execute_walk_to(self, parameters):
        """Execute walk to location action"""
        # This would typically call an action server for navigation
        if self.action_client.wait_for_service(timeout_sec=1.0):
            request = ExecuteAction.Request()
            request.action_name = 'navigate_to_pose'
            request.parameters = json.dumps(parameters)

            future = self.action_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
        else:
            self.get_logger().error('Action server not available')

def main(args=None):
    rclpy.init(args=args)
    node = LLMControlPipeline()

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

### 2. State Machine Integration Pattern

Implementing AI decision-making with state machines for humanoid behaviors:

```python
#!/usr/bin/env python3
"""
State Machine Integration for Humanoid AI
Combining AI decision-making with finite state machines for humanoid behaviors
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Imu
from enum import Enum
import time

class HumanoidState(Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
    MANIPULATING = "manipulating"
    BALANCING = "balancing"
    WAITING = "waiting"
    ERROR = "error"

class StateMachineAIAgent(Node):
    def __init__(self):
        super().__init__('state_machine_ai_agent')

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/ai_agent/status', 10)

        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        # Initialize state machine
        self.current_state = HumanoidState.IDLE
        self.previous_state = None
        self.state_entry_time = time.time()

        # State-specific data
        self.navigation_target = None
        self.manipulation_object = None
        self.balance_correction = None

        # Timer for state transitions
        self.state_timer = self.create_timer(0.1, self.state_machine_tick)

        # AI decision making timer
        self.ai_timer = self.create_timer(1.0, self.ai_decision_tick)

        self.get_logger().info('State Machine AI Agent initialized')

    def joint_callback(self, msg):
        """Handle joint state updates"""
        self.latest_joint_state = msg

    def imu_callback(self, msg):
        """Handle IMU updates for balance monitoring"""
        self.latest_imu = msg

        # Check if we need to transition to balancing state
        if self.current_state != HumanoidState.BALANCING:
            if self.should_enter_balancing_state(msg):
                self.transition_to_state(HumanoidState.BALANCING)

    def state_machine_tick(self):
        """Main state machine tick"""
        # Execute current state behavior
        if self.current_state == HumanoidState.IDLE:
            self.execute_idle_state()
        elif self.current_state == HumanoidState.NAVIGATING:
            self.execute_navigation_state()
        elif self.current_state == HumanoidState.MANIPULATING:
            self.execute_manipulation_state()
        elif self.current_state == HumanoidState.BALANCING:
            self.execute_balancing_state()
        elif self.current_state == HumanoidState.WAITING:
            self.execute_waiting_state()
        elif self.current_state == HumanoidState.ERROR:
            self.execute_error_state()

        # Check for state transitions based on current conditions
        self.check_state_transitions()

    def ai_decision_tick(self):
        """AI decision making that can trigger state changes"""
        # Analyze current situation and decide on state changes
        ai_decision = self.perform_ai_reasoning()

        if ai_decision and ai_decision != self.current_state:
            self.transition_to_state(ai_decision)

    def transition_to_state(self, new_state):
        """Safely transition to a new state"""
        if new_state != self.current_state:
            self.get_logger().info(f'State transition: {self.current_state.value} → {new_state.value}')

            # Execute exit behavior for current state
            self.exit_current_state()

            # Store previous state and update current
            self.previous_state = self.current_state
            self.current_state = new_state
            self.state_entry_time = time.time()

            # Execute entry behavior for new state
            self.enter_new_state()

    def exit_current_state(self):
        """Execute cleanup behavior when leaving current state"""
        if self.current_state == HumanoidState.NAVIGATING:
            # Stop movement when exiting navigation
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(stop_cmd)
        elif self.current_state == HumanoidState.BALANCING:
            # Reduce balance correction when exiting
            pass

    def enter_new_state(self):
        """Execute setup behavior when entering new state"""
        # Publish status update
        status_msg = String()
        status_msg.data = f"Entering {self.current_state.value} state"
        self.status_pub.publish(status_msg)

    def check_state_transitions(self):
        """Check conditions that might trigger state transitions"""
        if self.current_state == HumanoidState.BALANCING:
            # If balance is restored, return to previous state
            if self.is_balanced() and time.time() - self.state_entry_time > 2.0:
                if self.previous_state:
                    self.transition_to_state(self.previous_state)

        elif self.current_state == HumanoidState.NAVIGATING:
            # Check if navigation target reached
            if self.is_navigation_complete():
                self.transition_to_state(HumanoidState.IDLE)

    # State execution methods
    def execute_idle_state(self):
        """Behavior when in IDLE state"""
        # In idle state, just monitor sensors and wait for commands
        pass

    def execute_navigation_state(self):
        """Behavior when in NAVIGATION state"""
        if self.navigation_target:
            # Generate navigation commands towards target
            cmd = self.calculate_navigation_command(self.navigation_target)
            if cmd:
                self.cmd_vel_pub.publish(cmd)

    def execute_manipulation_state(self):
        """Behavior when in MANIPULATION state"""
        # Send manipulation commands
        # This would involve more complex joint control in practice
        pass

    def execute_balancing_state(self):
        """Behavior when in BALANCING state"""
        if self.latest_imu:
            # Calculate balance correction based on IMU data
            correction_cmd = self.calculate_balance_correction(self.latest_imu)
            if correction_cmd:
                self.cmd_vel_pub.publish(correction_cmd)

    def execute_waiting_state(self):
        """Behavior when in WAITING state"""
        # Just wait and monitor sensors
        pass

    def execute_error_state(self):
        """Behavior when in ERROR state"""
        # Emergency stop and safety procedures
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_cmd)

    def should_enter_balancing_state(self, imu_msg):
        """Check if we should enter balancing state based on IMU data"""
        # Calculate tilt from IMU orientation
        orientation = imu_msg.orientation
        # Convert quaternion to Euler angles to check tilt
        import math
        sinr_cosp = 2 * (orientation.w * orientation.x + orientation.y * orientation.z)
        cosr_cosp = 1 - 2 * (orientation.x * orientation.x + orientation.y * orientation.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # If tilt is too large, enter balancing state
        tilt_threshold = 0.3  # radians
        return abs(roll) > tilt_threshold or abs(yaw) > tilt_threshold

    def is_balanced(self):
        """Check if robot is currently balanced"""
        if not self.latest_imu:
            return False

        # Check if orientation is close to upright
        orientation = self.latest_imu.orientation
        return abs(orientation.z) > 0.7 and abs(orientation.w) > 0.7

    def is_navigation_complete(self):
        """Check if navigation target has been reached"""
        # This would involve checking current position vs. target
        # For now, return False to keep navigating
        return False

    def perform_ai_reasoning(self):
        """Perform AI reasoning to determine next state"""
        # This would typically call LLMs or other AI systems
        # For this example, we'll use simple rule-based reasoning

        # Example: If we receive a navigation command externally, switch to NAVIGATING
        # In practice, this would analyze multiple factors and make complex decisions
        return self.current_state  # No state change in this simple example

def main(args=None):
    rclpy.init(args=args)
    node = StateMachineAIAgent()

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

## Command Pipeline Implementation

### 1. LLM-to-Action Planner-to-ROS Controller Pipeline

Creating a complete pipeline from LLM to ROS controller:

```python
#!/usr/bin/env python3
"""
Complete LLM → Action Planner → ROS Controller Pipeline
Full pipeline connecting language models to robot controllers
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
from humanoid_msgs.action import NavigateToPose, ManipulateObject
from geometry_msgs.msg import Pose
import openai
import json
import asyncio
from typing import Dict, List, Any

class LLMActionPipeline(Node):
    def __init__(self):
        super().__init__('llm_action_pipeline')

        # Publishers for status updates
        self.status_pub = self.create_publisher(String, '/ai_pipeline/status', 10)

        # Action clients for robot execution
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.manip_client = ActionClient(self, ManipulateObject, '/manipulate_object')

        # Subscription for high-level commands
        self.high_level_cmd_sub = self.create_subscription(
            String, '/high_level_commands', self.high_level_command_callback, 10
        )

        # Initialize LLM client
        try:
            self.openai_client = openai.OpenAI(api_key=self.get_parameter('openai.api_key').value)
        except:
            self.get_logger().warn('OpenAI API key not configured')
            self.openai_client = None

        self.get_logger().info('LLM → Action Planner → ROS Controller Pipeline initialized')

    def high_level_command_callback(self, msg):
        """Process high-level command through the full pipeline"""
        command = msg.data

        try:
            # Step 1: Use LLM to decompose high-level command
            task_plan = self.llm_decompose_command(command)

            if task_plan:
                # Step 2: Execute the task plan through ROS controllers
                self.execute_task_plan(task_plan)
        except Exception as e:
            self.get_logger().error(f'Error in command pipeline: {str(e)}')
            self.publish_status(f'Error: {str(e)}')

    def llm_decompose_command(self, command: str) -> List[Dict[str, Any]]:
        """Use LLM to decompose high-level command into task plan"""
        if not self.openai_client:
            # Mock decomposition if LLM is not available
            return self.mock_decompose_command(command)

        prompt = f"""
        You are a task planner for a humanoid robot. Decompose the following high-level command into a sequence of specific robotic tasks:

        Command: "{command}"

        Respond in JSON format with an array of tasks:
        [
            {{
                "id": "unique_task_id",
                "type": "navigation|manipulation|perception|locomotion",
                "description": "What the task does",
                "parameters": {{"param1": "value1", "param2": "value2"}},
                "dependencies": ["task_id_1", "task_id_2"]  // Optional: tasks that must complete first
            }}
        ]

        For navigation tasks, include pose parameters.
        For manipulation tasks, include object and grasp parameters.
        For perception tasks, include sensor and detection parameters.
        For locomotion tasks, include gait and balance parameters.
        """

        try:
            response = self.openai_client.chat.completions.create(
                model="gpt-4-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.1
            )

            response_text = response.choices[0].message.content.strip()
            json_match = re.search(r'\[.*\]', response_text, re.DOTALL)
            if json_match:
                task_plan = json.loads(json_match.group())
                self.get_logger().info(f'Decomposed command into {len(task_plan)} tasks')
                return task_plan

        except Exception as e:
            self.get_logger().error(f'LLM decomposition failed: {str(e)}')

        return []  # Return empty plan if decomposition fails

    def mock_decompose_command(self, command: str) -> List[Dict[str, Any]]:
        """Mock command decomposition for testing"""
        command_lower = command.lower()

        if 'go to' in command_lower or 'navigate to' in command_lower:
            # Extract location
            location = 'kitchen'
            if 'kitchen' in command_lower:
                location = 'kitchen'
            elif 'living room' in command_lower:
                location = 'living_room'
            elif 'bedroom' in command_lower:
                location = 'bedroom'

            return [{
                "id": "nav_task_1",
                "type": "navigation",
                "description": f"Navigate to {location}",
                "parameters": {
                    "location": location,
                    "target_pose": {"x": 2.5, "y": 3.0, "theta": 0.0}
                },
                "dependencies": []
            }]
        elif 'pick up' in command_lower or 'grasp' in command_lower:
            # Extract object
            obj = 'object'
            if 'red ball' in command_lower:
                obj = 'red_ball'
            elif 'blue cup' in command_lower:
                obj = 'blue_cup'
            elif 'green box' in command_lower:
                obj = 'green_box'

            return [
                {
                    "id": "nav_task_1",
                    "type": "navigation",
                    "description": f"Navigate to {obj}",
                    "parameters": {
                        "target_pose": {"x": 1.0, "y": 1.0, "theta": 0.0}
                    },
                    "dependencies": []
                },
                {
                    "id": "manip_task_1",
                    "type": "manipulation",
                    "description": f"Grasp {obj}",
                    "parameters": {
                        "object": obj,
                        "grasp_type": "top_grasp"
                    },
                    "dependencies": ["nav_task_1"]
                }
            ]
        else:
            return [{
                "id": "wait_task_1",
                "type": "locomotion",
                "description": "Wait for clarification",
                "parameters": {},
                "dependencies": []
            }]

    def execute_task_plan(self, task_plan: List[Dict[str, Any]]):
        """Execute the task plan using ROS controllers"""
        self.get_logger().info(f'Executing task plan with {len(task_plan)} tasks')

        # Execute tasks sequentially for now (could be made more sophisticated)
        for task in task_plan:
            self.execute_single_task(task)

    def execute_single_task(self, task: Dict[str, Any]):
        """Execute a single task based on its type"""
        task_type = task.get('type')
        params = task.get('parameters', {})

        if task_type == 'navigation':
            self.execute_navigation_task(params)
        elif task_type == 'manipulation':
            self.execute_manipulation_task(params)
        elif task_type == 'perception':
            self.execute_perception_task(params)
        elif task_type == 'locomotion':
            self.execute_locomotion_task(params)
        else:
            self.get_logger().warn(f'Unknown task type: {task_type}')
            self.publish_status(f'Unknown task type: {task_type}')

    def execute_navigation_task(self, params: Dict[str, Any]):
        """Execute navigation task using NavigateToPose action"""
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Navigation action server not available')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = Pose()
        goal_msg.pose.position.x = params.get('target_pose', {}).get('x', 0.0)
        goal_msg.pose.position.y = params.get('target_pose', {}).get('y', 0.0)
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.z = params.get('target_pose', {}).get('theta', 0.0)
        goal_msg.pose.orientation.w = 1.0

        self.publish_status(f'Navigating to position ({goal_msg.pose.position.x}, {goal_msg.pose.position.y})')

        # Send goal and wait for result
        future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

    def execute_manipulation_task(self, params: Dict[str, Any]):
        """Execute manipulation task using ManipulateObject action"""
        if not self.manip_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Manipulation action server not available')
            return

        goal_msg = ManipulateObject.Goal()
        goal_msg.object_name = params.get('object', 'unknown')
        goal_msg.grasp_type = params.get('grasp_type', 'pinch_grasp')

        self.publish_status(f'Manipulating object: {goal_msg.object_name}')

        # Send goal and wait for result
        future = self.manip_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

    def execute_perception_task(self, params: Dict[str, Any]):
        """Execute perception task"""
        # Perception tasks would typically involve sensor activation and data processing
        sensor_type = params.get('sensor', 'camera')
        detection_type = params.get('detection', 'object')

        self.publish_status(f'Performing {detection_type} detection using {sensor_type}')

    def execute_locomotion_task(self, params: Dict[str, Any]):
        """Execute locomotion task"""
        # Locomotion tasks would involve gait control and balance
        gait_type = params.get('gait', 'walk')

        self.publish_status(f'Executing {gait_type} locomotion')

    def publish_status(self, status: str):
        """Publish status update"""
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LLMActionPipeline()

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

## Quality Validation for rclpy Integration

### 1. Code Quality Standards

For Python agent integration with ROS 2:

- Use proper type hints for all functions and methods
- Follow ROS 2 Python style guidelines
- Implement proper error handling for all ROS operations
- Use appropriate QoS profiles for different communication needs
- Include comprehensive logging for debugging
- Implement parameter validation for all inputs

### 2. Testing Approaches

```python
#!/usr/bin/env python3
# test_rclpy_integration.py
import unittest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool

class TestRclpyIntegration(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('test_rclpy_integration')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        # Test publisher and subscriber
        self.test_publisher = self.node.create_publisher(String, '/test_topic', 10)
        self.received_messages = []

        def msg_callback(msg):
            self.received_messages.append(msg.data)

        self.test_subscriber = self.node.create_subscription(
            String, '/test_topic', msg_callback, 10
        )

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_basic_publish_subscribe(self):
        """Test basic publish/subscribe functionality"""
        test_msg = String()
        test_msg.data = "test_message"

        self.test_publisher.publish(test_msg)

        # Allow brief time for message to be processed
        self.executor.spin_once(timeout_sec=0.1)

        self.assertEqual(len(self.received_messages), 1)
        self.assertEqual(self.received_messages[0], "test_message")

    def test_parameter_declaration(self):
        """Test parameter declaration and access"""
        self.node.declare_parameter('test_param', 'default_value')
        param_value = self.node.get_parameter('test_param').value
        self.assertEqual(param_value, 'default_value')

    def test_service_call(self):
        """Test service call functionality"""
        # Create a test service
        service_called = False
        def service_callback(request, response):
            nonlocal service_called
            service_called = True
            response.success = True
            response.message = "Service called successfully"
            return response

        service = self.node.create_service(SetBool, '/test_service', service_callback)

        # Create a client and call the service
        client = self.node.create_client(SetBool, '/test_service')

        # Wait for service to be available
        if client.wait_for_service(timeout_sec=1.0):
            request = SetBool.Request()
            request.data = True
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)

            self.assertTrue(service_called)

if __name__ == '__main__':
    unittest.main()
```

### 3. Performance Validation

```bash
# Performance testing for rclpy integration
# Test message throughput
ros2 run demo_nodes_py talker --ros-args -p publish_frequency:=100.0

# Monitor message rates
ros2 topic hz /chatter

# Test node startup time
time ros2 run your_package your_ai_node

# Monitor resource usage
htop  # or other system monitoring tools
```

## Integration with Book Content Structure

### 1. Docusaurus Integration

The rclpy content should be properly integrated into the Docusaurus structure:

```javascript
// In docs/sidebars.js
module.exports = {
  vlaIntegration: [
    'modules/module-4-vla/introduction',
    {
      type: 'category',
      label: 'Foundations of VLA Systems',
      items: [
        'modules/module-4-vla/foundations/vla-definition',
        'modules/module-4-vla/foundations/llm-integration',
        'modules/module-4-vla/foundations/components-required',
        'modules/module-4-vla/foundations/classical-vs-vla'
      ]
    },
    {
      type: 'category',
      label: 'Voice-to-Action Pipeline',
      items: [
        'modules/module-4-vla/voice-to-action/voice-processing',
        'modules/module-4-vla/voice-to-action/language-understanding',
        'modules/module-4-vla/voice-to-action/ros-integration',
        'modules/module-4-vla/voice-to-action/behavior-trees',
        'modules/module-4-vla/voice-to-action/rclpy-integration'  // Our current content
      ]
    },
    {
      type: 'category',
      label: 'Integration & Capstone',
      items: [
        'modules/module-4-vla/integration/system-coordination',
        'modules/module-4-vla/integration/capstone-spec',
        'modules/module-4-vla/integration/implementation-guide'
      ]
    }
  ]
};
```

### 2. Cross-Module Linking

Ensure proper linking to other modules:

```md
For more information on ROS 2 fundamentals, see [Module 1: ROS 2 Nervous System](../module-1-ros2/introduction.md).

For simulation environments, see [Module 2: Digital Twin](../module-2-simulation/introduction.md).

For navigation systems, see [Module 3: Navigation & Perception](../module-3-navigation/introduction.md).
```

## Troubleshooting Common Issues

### 1. rclpy Threading Issues

```python
# Problem: Blocking operations in callbacks
def bad_callback(self, msg):
    time.sleep(5)  # Blocks the entire node!

# Solution: Use threading for long operations
def good_callback(self, msg):
    # Submit long operations to a thread pool
    future = self.executor.submit(self.long_operation, msg)
    future.add_done_callback(self.operation_complete_callback)

def long_operation(self, msg):
    # Do time-consuming work here
    time.sleep(5)
    return "result"

def operation_complete_callback(self, future):
    result = future.result()
    # Handle result in the main thread
```

### 2. Memory Management

```python
# Proper cleanup in node destruction
def destroy_node(self):
    # Cancel any timers
    if hasattr(self, 'timer') and self.timer is not None:
        self.timer.cancel()

    # Destroy publishers and subscribers
    if hasattr(self, 'publisher') and self.publisher is not None:
        self.destroy_publisher(self.publisher)

    if hasattr(self, 'subscriber') and self.subscriber is not None:
        self.destroy_subscription(self.subscriber)

    # Call parent destroy
    super().destroy_node()
```

## Next Steps

1. Implement the rclpy integration examples in your humanoid simulation
2. Test AI agent communication with ROS 2 systems
3. Create custom message types for humanoid-specific commands
4. Develop advanced AI reasoning capabilities using the patterns shown
5. Integrate with the RAG system for contextual AI responses
6. Validate all communication patterns with the quality standards defined in the Constitution

## References

- ROS 2 Python Client Library (rclpy): https://docs.ros.org/en/humble/p/rclpy/
- ROS 2 Python Style Guide: https://docs.ros.org/en/humble/How-To-Guides/PyStyleGuide.html
- ROS 2 Actions in Python: https://docs.ros.org/en/humble/Tutorials/Actions/Writing-a-simple-action-server-python.html
- OpenAI Python Library: https://platform.openai.com/docs/libraries/python-library
- Asyncio in ROS 2: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Async-Callbacks-In-Python.html