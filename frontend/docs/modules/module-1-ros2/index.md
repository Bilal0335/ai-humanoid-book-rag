---
sidebar_position: 1
---

# Module 1: ROS 2 Nervous System

## Overview

Module 1 establishes the foundational concepts of ROS 2 as the middleware "nervous system" for humanoid robots. This module covers communication primitives, integration patterns, and the essential components needed to connect AI agents with physical robot systems.

## Learning Objectives

By the end of this module, students will be able to:

- Understand the fundamentals of ROS 2 as a middleware system for humanoid robotics
- Implement ROS 2 communication patterns (nodes, topics, services, actions)
- Connect Python AI agents to ROS 2 using rclpy
- Design humanoid models using URDF for robot representation
- Build command pipelines connecting LLMs to robot controllers

## Module Structure

This module is organized into three main chapters:

### Chapter 1: Foundations of ROS 2 as a Middleware System
- What is ROS 2 and why it matters for humanoid robotics
- Differences between ROS 1 and ROS 2
- ROS 2 architecture layers (DDS, rmw, executors)
- Installation and workspace setup with colcon
- Lifecycle of a humanoid control system

### Chapter 2: ROS 2 Communication Patterns
- Nodes: Structure, callbacks, and spin mechanisms
- Topics: Publishers, subscribers, and QoS profiles
- Services vs Actions: When to use which
- Communication patterns in humanoid robots
- Building controller nodes for humanoid joints
- Best practices for real-time control

### Chapter 3: Python Agent Integration and URDF
- Using rclpy to connect Python agents to ROS 2
- Designing AI to ROS 2 control pipelines
- Implementing command pipelines (LLM → Action Planner → ROS Controller)
- URDF: Structure, links, joints, and inertial parameters
- Creating humanoid URDF models
- Launch files for multi-node humanoid systems

## Prerequisites

Before starting this module, students should have:

- Basic knowledge of Python programming
- Understanding of fundamental robotics concepts
- Familiarity with command-line tools
- Basic understanding of middleware concepts

## Integration with Other Modules

This module provides the foundational ROS 2 knowledge that will be used throughout the course:

- **Module 2 Foundation**: Uses ROS 2 communication for simulation
- **Module 3 Integration**: Connects perception systems via ROS 2
- **Module 4 Pipeline**: Enables VLA systems through ROS 2 control

## Technical Requirements

- Ubuntu 22.04 or WSL2
- ROS 2 Humble Hawksbill
- Python 3.8-3.10
- Basic development tools (colcon, etc.)

## Getting Started

Begin with Chapter 1 to understand the ROS 2 foundations, then progress through communication patterns and integration techniques to build a complete understanding of ROS 2 in the context of humanoid robotics.