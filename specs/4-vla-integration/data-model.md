# Data Model: Vision-Language-Action (VLA) Integration Module

**Date**: 2025-12-11
**Feature**: Vision-Language-Action (VLA) Integration Module
**Status**: Complete

## Overview

This document defines the data models for the Vision-Language-Action (VLA) Integration module, focusing on entities that connect natural language understanding with robotic execution. The models include representations for voice commands, language processing, action plans, and robotic execution states.

## Core Entities

### 1. VoiceCommand
**Description**: Represents a voice command received by the VLA system

**Fields**:
- `id` (string): Unique identifier for the command
- `audio_data` (binary): Raw audio data or path to audio file
- `transcript` (string): Text transcription of the spoken command
- `confidence_score` (float): Confidence score of the speech recognition (0.0-1.0)
- `timestamp` (datetime): When the command was received
- `language_code` (string): Language of the spoken command (e.g., "en-US")
- `noise_level` (float): Estimated noise level during recording (0.0-1.0)
- `processing_status` (enum): Status of processing (received, transcribed, understood, executed, failed)

**Relationships**:
- Connected to one `LanguageUnderstanding` result
- Connected to one `ActionPlan` (after processing)
- Connected to `UserSession` for context

### 2. LanguageUnderstanding
**Description**: Represents the processed understanding of a voice command

**Fields**:
- `id` (string): Unique identifier for the understanding result
- `voice_command_id` (string): Reference to the original voice command
- `intent_classification` (string): Classified intent of the command
- `entities_extracted` (array): Extracted named entities (objects, locations, people)
- `structured_goal` (object): Structured representation of the desired goal
- `confidence_score` (float): Confidence in the understanding (0.0-1.0)
- `task_decomposition` (array): Breakdown of the command into subtasks
- `context_variables` (object): Context variables extracted from command
- `created_at` (datetime): Timestamp of creation

**Relationships**:
- Belongs to one `VoiceCommand`
- Connected to one `ActionPlan` (for execution)
- Connected to `KnowledgeGraph` for entity resolution

### 3. ActionPlan
**Description**: Represents a structured plan of actions to execute a command

**Fields**:
- `id` (string): Unique identifier for the action plan
- `language_understanding_id` (string): Reference to the understanding result
- `plan_type` (enum): Type of plan (sequential, parallel, conditional, hierarchical)
- `tasks_sequence` (array): Sequence of tasks to execute
- `execution_context` (object): Context needed for execution (locations, objects, constraints)
- `safety_constraints` (array): Safety constraints to consider during execution
- `success_criteria` (array): Conditions that define successful execution
- `fallback_behaviors` (array): Behaviors to execute if primary plan fails
- `estimated_duration` (integer): Estimated time to execute in seconds
- `created_at` (datetime): Timestamp of creation

**Relationships**:
- Belongs to one `LanguageUnderstanding`
- Connected to multiple `RosActionCall` entries
- Connected to `BehaviorTree` for orchestration

### 4. RosActionCall
**Description**: Represents a specific ROS 2 action call within an action plan

**Fields**:
- `id` (string): Unique identifier for the action call
- `action_plan_id` (string): Reference to the parent action plan
- `action_name` (string): Name of the ROS action (e.g., "move_base_msgs/MoveBase")
- `action_server` (string): Name of the action server to call
- `goal_parameters` (object): Parameters for the action goal
- `feedback_callback` (string): Callback function for feedback handling
- `timeout_seconds` (integer): Timeout for the action execution
- `retry_attempts` (integer): Number of retry attempts if failed
- `priority` (integer): Priority level for execution (1-10)
- `preconditions` (array): Conditions that must be met before execution
- `postconditions` (array): Expected conditions after execution
- `created_at` (datetime): Timestamp of creation

**Relationships**:
- Belongs to one `ActionPlan`
- Connected to `RosNode` for execution
- Connected to `ExecutionLog` for tracking

### 5. BehaviorTree
**Description**: Represents a behavior tree for orchestrating complex tasks

**Fields**:
- `id` (string): Unique identifier for the behavior tree
- `name` (string): Name of the behavior tree
- `root_node` (object): Root node of the behavior tree structure
- `nodes` (array): Array of all nodes in the tree
- `blackboard` (object): Shared data structure for node communication
- `execution_frequency_hz` (float): Frequency of tree execution
- `timeout_seconds` (integer): Overall timeout for tree execution
- `recovery_strategies` (array): Strategies for handling failures
- `created_at` (datetime): Timestamp of creation
- `updated_at` (datetime): Timestamp of last update

**Relationships**:
- Connected to multiple `ActionPlan` entries
- Connected to multiple `BehaviorTreeNode` entries
- Connected to `ExecutionLog` for tracking

### 6. BehaviorTreeNode
**Description**: Represents a single node in a behavior tree

**Fields**:
- `id` (string): Unique identifier for the node
- `tree_id` (string): Reference to the parent behavior tree
- `node_type` (enum): Type of node (sequence, selector, condition, action, decorator)
- `node_name` (string): Name of the node
- `node_parameters` (object): Parameters for the node
- `children` (array): IDs of child nodes (for composite nodes)
- `tick_function` (string): Function to execute when node is ticked
- `status` (enum): Current status (success, failure, running, idle)
- `created_at` (datetime): Timestamp of creation

**Relationships**:
- Belongs to one `BehaviorTree`
- Connected to other `BehaviorTreeNode` entries (parent-child relationships)

### 7. ExecutionLog
**Description**: Tracks the execution of action plans and behavior trees

**Fields**:
- `id` (string): Unique identifier for the log entry
- `entity_id` (string): ID of the entity being executed (action plan or behavior tree)
- `entity_type` (enum): Type of entity (action_plan, behavior_tree)
- `status` (enum): Execution status (started, running, succeeded, failed, cancelled)
- `timestamp` (datetime): When the event occurred
- `details` (object): Additional details about the execution
- `error_message` (string): Error message if execution failed
- `duration_ms` (integer): Duration of execution in milliseconds
- `feedback_data` (object): Feedback received during execution

**Relationships**:
- Connected to `ActionPlan` or `BehaviorTree` (via entity_id)
- Connected to `RosActionCall` for specific action tracking

### 8. KnowledgeGraph
**Description**: Represents the knowledge base used for understanding and reasoning

**Fields**:
- `id` (string): Unique identifier for the knowledge graph
- `entities` (array): Named entities with properties and relationships
- `relations` (array): Relationships between entities
- `ontologies` (array): Formal ontologies defining concepts
- `context_frames` (array): Context-specific knowledge frames
- `update_timestamp` (datetime): When the graph was last updated
- `confidence_threshold` (float): Minimum confidence for entity linking (0.0-1.0)

**Relationships**:
- Connected to multiple `LanguageUnderstanding` entries
- Connected to `EntityResolution` for linking entities

### 9. EntityResolution
**Description**: Maps recognized entities to specific objects in the environment

**Fields**:
- `id` (string): Unique identifier for the entity resolution
- `recognized_entity` (string): Entity as recognized in the command
- `resolved_object_id` (string): ID of the actual object in the environment
- `confidence_score` (float): Confidence in the resolution (0.0-1.0)
- `resolution_method` (enum): Method used for resolution (visual, spatial, historical)
- `alternative_objects` (array): Alternative object candidates with scores
- `spatial_location` (object): Spatial location of the resolved object
- `visual_features` (array): Visual features used for resolution
- `created_at` (datetime): Timestamp of creation

**Relationships**:
- Connected to `LanguageUnderstanding` for entity linking
- Connected to `EnvironmentObject` for spatial mapping

### 10. EnvironmentObject
**Description**: Represents objects in the robot's environment

**Fields**:
- `id` (string): Unique identifier for the environment object
- `name` (string): Human-readable name of the object
- `category` (string): Category of the object (furniture, tool, food, etc.)
- `pose` (object): Position and orientation in the environment
- `dimensions` (object): Physical dimensions of the object
- `manipulation_properties` (object): Properties for manipulation (grasp points, weight, etc.)
- `visibility_status` (enum): Visibility status (visible, occluded, out_of_view)
- `tracking_confidence` (float): Confidence in object tracking (0.0-1.0)
- `last_seen_timestamp` (datetime): When the object was last detected
- `semantic_properties` (object): Semantic properties (movable, graspable, etc.)

**Relationships**:
- Connected to multiple `EntityResolution` entries
- Connected to `PerceptionData` for tracking

### 11. UserSession
**Description**: Represents a session of interaction with the VLA system

**Fields**:
- `id` (string): Unique identifier for the session
- `user_id` (string): Identifier for the user (optional)
- `start_timestamp` (datetime): When the session started
- `end_timestamp` (datetime): When the session ended (null if active)
- `session_context` (object): Context maintained during the session
- `recent_interactions` (array): Recent voice commands and responses
- `dialogue_state` (object): Current state of the dialogue
- `preferences` (object): User preferences and settings
- `session_metrics` (object): Metrics for the session (success rate, etc.)

**Relationships**:
- Connected to multiple `VoiceCommand` entries
- Connected to multiple `LanguageUnderstanding` results

### 12. VlaPipeline
**Description**: Represents the complete VLA pipeline configuration

**Fields**:
- `id` (string): Unique identifier for the pipeline
- `name` (string): Name of the pipeline configuration
- `speech_recognition_config` (object): Configuration for speech recognition
- `language_understanding_config` (object): Configuration for language processing
- `action_planning_config` (object): Configuration for action planning
- `execution_config` (object): Configuration for action execution
- `safety_settings` (object): Safety constraints and limits
- `performance_metrics` (object): Expected performance metrics
- `created_at` (datetime): Timestamp of creation
- `updated_at` (datetime): Timestamp of last update

**Relationships**:
- Connected to multiple `VoiceCommand` entries processed by this pipeline
- Connected to `ExecutionLog` for pipeline-wide tracking

## Validation Rules

### VoiceCommand Validation
- `transcript` must not be empty
- `confidence_score` must be between 0.0 and 1.0
- `processing_status` must be one of the defined enum values
- `timestamp` must be a valid datetime

### LanguageUnderstanding Validation
- `structured_goal` must be a valid object with required fields
- `confidence_score` must be between 0.0 and 1.0
- `voice_command_id` must reference an existing voice command
- `task_decomposition` must contain at least one task

### ActionPlan Validation
- `plan_type` must be one of the defined enum values
- `tasks_sequence` must contain at least one task
- `estimated_duration` must be a positive integer
- `action_plan_id` must reference an existing language understanding

### RosActionCall Validation
- `action_name` must be a valid ROS action name
- `action_server` must reference an existing action server
- `timeout_seconds` must be a positive integer
- `priority` must be between 1 and 10

### BehaviorTree Validation
- `root_node` must be a valid behavior tree node
- `execution_frequency_hz` must be positive
- `timeout_seconds` must be positive

## State Transitions

### VoiceCommand States
- `received` → `transcribed`: When speech-to-text processing completes
- `transcribed` → `understood`: When language understanding completes
- `understood` → `executed`: When action plan completes successfully
- `understood` → `failed`: When action plan execution fails

### ActionPlan States
- `created` → `scheduled`: When plan is scheduled for execution
- `scheduled` → `executing`: When plan execution begins
- `executing` → `succeeded`: When all tasks complete successfully
- `executing` → `failed`: When execution encounters unrecoverable error
- `executing` → `cancelled`: When execution is manually cancelled

### BehaviorTree States
- `idle` → `running`: When tree execution starts
- `running` → `success`: When tree completes successfully
- `running` → `failure`: When tree encounters failure condition
- `running` → `running`: When tree continues to next cycle