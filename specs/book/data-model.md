# Data Model: AI/Spec-driven Book Creation with Docusaurus

**Date**: 2025-12-11
**Feature**: AI/Spec-driven Book Creation with Docusaurus
**Status**: Complete

## Overview

This document defines the data models for the Physical AI & Humanoid Robotics book project, focusing on content organization and structure for Docusaurus-based book creation with integrated RAG functionality.

## Core Entities

### 1. BookContent
**Description**: Represents the content of the book organized by modules and sections

**Fields**:
- `id` (string): Unique identifier for the content piece
- `module_id` (string): Reference to the module this content belongs to
- `section_id` (string): Reference to the section within the module
- `title` (string): Title of the content piece
- `content_type` (enum): Type of content (text, code, diagram, video, exercise, deep-dive)
- `content` (string): The actual content in MDX format
- `embedded_content` (string): Content pre-processed for embedding (stripped of formatting)
- `page_reference` (string): Reference for citation purposes (e.g., "Module 2, Section 3.2")
- `tags` (array): Array of tags for categorization and search
- `created_at` (datetime): Timestamp of creation
- `updated_at` (datetime): Timestamp of last update
- `version` (string): Version of the content
- `difficulty_level` (enum): Difficulty level (beginner, intermediate, advanced)
- `estimated_reading_time_minutes` (integer): Estimated time to read/understand the content

**Relationships**:
- Belongs to one `Module`
- Belongs to one `Section`
- Connected to multiple `EmbeddingVector` entries
- Connected to multiple `Citation` entries

### 2. Module
**Description**: Represents a major module in the book (e.g., ROS 2, Simulation, Navigation, VLA)

**Fields**:
- `id` (string): Unique identifier for the module
- `name` (string): Name of the module (e.g., "ROS 2 Nervous System")
- `description` (string): Brief description of the module
- `order` (integer): Order of the module in the book sequence (1-5)
- `status` (enum): Status of the module (draft, review, published)
- `created_at` (datetime): Timestamp of creation
- `updated_at` (datetime): Timestamp of last update
- `estimated_completion_hours` (integer): Estimated hours to complete the module
- `prerequisites` (array): Array of prerequisite modules or concepts

**Relationships**:
- Contains multiple `Section` entries
- Connected to multiple `BookContent` entries
- Connected to one `UnifiedHumanoidModel` (for cross-module consistency)

### 3. Section
**Description**: Represents a section within a module

**Fields**:
- `id` (string): Unique identifier for the section
- `module_id` (string): Reference to the parent module
- `title` (string): Title of the section
- `description` (string): Brief description of the section
- `order` (integer): Order of the section within the module
- `created_at` (datetime): Timestamp of creation
- `updated_at` (datetime): Timestamp of last update
- `learning_objectives` (array): Array of learning objectives for the section
- `prerequisites` (array): Array of prerequisite concepts or sections

**Relationships**:
- Belongs to one `Module`
- Connected to multiple `BookContent` entries
- Connected to multiple `CrossReference` entries (for inter-section links)

### 4. EmbeddingVector
**Description**: Represents vector embeddings of book content for RAG retrieval

**Fields**:
- `id` (string): Unique identifier for the embedding
- `content_id` (string): Reference to the associated book content
- `vector` (array): The actual embedding vector
- `chunk_text` (string): The text chunk that was embedded
- `chunk_metadata` (object): Metadata about the chunk (page_reference, module, section, difficulty, etc.)
- `created_at` (datetime): Timestamp of creation
- `embedding_model` (string): Model used for creating the embedding

**Relationships**:
- Belongs to one `BookContent`
- Used by `ChatSession` for retrieval

### 5. ChatSession
**Description**: Represents a conversation session with the RAG chatbot

**Fields**:
- `id` (string): Unique identifier for the session
- `user_id` (string): Identifier for the user (optional for anonymous)
- `created_at` (datetime): Timestamp of session creation
- `updated_at` (datetime): Timestamp of last interaction
- `session_metadata` (object): Additional metadata about the session

**Relationships**:
- Contains multiple `ChatMessage` entries
- Uses multiple `RetrievedContent` entries

### 6. ChatMessage
**Description**: Represents a single message in a chat session

**Fields**:
- `id` (string): Unique identifier for the message
- `session_id` (string): Reference to the parent session
- `role` (enum): Role of the message (user, assistant)
- `content` (string): The actual message content
- `timestamp` (datetime): When the message was created
- `citations` (array): Array of content references used in the response
- `confidence_score` (float): Confidence score of the response (0-1)

**Relationships**:
- Belongs to one `ChatSession`
- Connected to multiple `RetrievedContent` entries (for assistant messages)

### 7. RetrievedContent
**Description**: Represents content retrieved from the book for a specific query

**Fields**:
- `id` (string): Unique identifier for the retrieval
- `session_id` (string): Reference to the chat session
- `query` (string): The original query that triggered the retrieval
- `content_id` (string): Reference to the retrieved book content
- `similarity_score` (float): Similarity score from vector search
- `retrieved_at` (datetime): When the content was retrieved
- `context_window` (string): The context provided to the LLM
- `chunk_start_index` (integer): Start index of the content chunk in the original document

**Relationships**:
- Belongs to one `ChatSession`
- Connected to one `BookContent`
- Connected to one `EmbeddingVector`

### 8. UnifiedHumanoidModel
**Description**: Represents the consistent humanoid robot model used across all modules

**Fields**:
- `id` (string): Unique identifier for the model
- `name` (string): Name of the model (e.g., "EducationalHumanoid")
- `urdf_path` (string): Path to the URDF file
- `sdf_path` (string): Path to the SDF file (if applicable)
- `mesh_paths` (array): Array of paths to mesh files
- `joint_configurations` (object): Default joint configurations
- `physical_properties` (object): Mass, inertia, friction properties
- `sensor_configurations` (object): Sensor placements and types
- `created_at` (datetime): Timestamp of creation
- `updated_at` (datetime): Timestamp of last update
- `transform_guides` (object): Guides for transforming the model for different simulation environments

**Relationships**:
- Used in multiple `SimulationEnvironment` entries
- Referenced by all modules that use the humanoid model

### 9. SimulationEnvironment
**Description**: Represents a simulation environment configuration

**Fields**:
- `id` (string): Unique identifier for the environment
- `name` (string): Name of the environment (e.g., "GazeboWorld1")
- `type` (enum): Type of simulation (gazebo, unity, isaac)
- `configuration` (object): Environment-specific configuration
- `world_file_path` (string): Path to the world/simulation file
- `robot_spawn_config` (object): Configuration for robot spawning
- `created_at` (datetime): Timestamp of creation
- `updated_at` (datetime): Timestamp of last update
- `performance_metrics` (object): Metrics for simulation performance

**Relationships**:
- Uses one `UnifiedHumanoidModel`
- Associated with multiple `BookContent` entries (tutorials)

### 10. UserProgress
**Description**: Tracks user progress through the book modules

**Fields**:
- `id` (string): Unique identifier for the progress record
- `user_id` (string): Identifier for the user
- `module_id` (string): Reference to the module
- `section_id` (string): Reference to the section (optional)
- `completed` (boolean): Whether the module/section is completed
- `progress_percentage` (float): Percentage of completion
- `last_accessed_at` (datetime): When the user last accessed this content
- `time_spent_seconds` (integer): Time spent on the content
- `notes` (string): User notes about the content
- `assessment_scores` (array): Array of scores from exercises/assessments

**Relationships**:
- Connected to one `Module` (and optionally one `Section`)
- Connected to one `User` (user_id)

### 11. Citation
**Description**: Represents a citation or reference within content

**Fields**:
- `id` (string): Unique identifier for the citation
- `content_id` (string): Reference to the content block containing the citation
- `citation_type` (enum): Type of citation (academic, documentation, online, book, ros-docs, gazebo-docs, unity-docs, isaac-docs)
- `author` (string): Author(s) of the cited work
- `title` (string): Title of the cited work
- `source` (string): Source of the citation (journal, website, book, documentation)
- `url` (string): URL to the cited resource (if applicable)
- `access_date` (date): Date when the resource was accessed
- `reference_number` (integer): Number in the reference list
- `apa_format` (string): Full citation in APA format
- `created_at` (datetime): Timestamp of creation

**Relationships**:
- Belongs to one `BookContent`
- Connected to `ValidationCheck` for accuracy verification

### 12. CrossReference
**Description**: Represents a reference from one section to another

**Fields**:
- `id` (string): Unique identifier for the cross-reference
- `from_section_id` (string): Section containing the reference
- `to_section_id` (string): Section being referenced
- `reference_type` (enum): Type of reference (see-also, prerequisite, extension, alternative-approach)
- `description` (string): Description of the reference relationship
- `link_text` (string): Text to display for the link
- `created_at` (datetime): Timestamp of creation

**Relationships**:
- Connected to two `Section` entries (source and target)

### 13. ValidationCheck
**Description**: Represents a validation check for content accuracy and reproducibility

**Fields**:
- `id` (string): Unique identifier for the validation check
- `content_id` (string): Reference to the content being validated
- `check_type` (enum): Type of validation (technical-accuracy, reproducibility, citation-accuracy, constitution-compliance)
- `validator` (string): Who performed the validation (person or automated tool)
- `status` (enum): Status of validation (pending, passed, failed, needs-update)
- `notes` (string): Notes about the validation process
- `validation_date` (datetime): When the validation was performed
- `next_validation_date` (datetime): When to revalidate (if applicable)

**Relationships**:
- Connected to one `BookContent`
- Connected to `Constitution` for compliance checking

## Validation Rules

### BookContent Validation
- `title` must not be empty
- `content` must be valid MDX format
- `content_type` must be one of the defined enum values
- `module_id` must reference an existing module
- `page_reference` must follow format "Module X, Section Y.Z"
- `difficulty_level` must be one of beginner, intermediate, or advanced
- `estimated_reading_time_minutes` must be a positive integer

### Module Validation
- `name` must be unique
- `order` must be between 1 and 5
- `status` must be one of draft, review, or published
- `estimated_completion_hours` must be a positive number

### EmbeddingVector Validation
- `vector` must have consistent dimensions (1536 for OpenAI text-embedding-3-small)
- `content_id` must reference an existing book content
- `chunk_text` must not exceed token limits for embedding model

### ChatMessage Validation
- `role` must be either 'user' or 'assistant'
- `session_id` must reference an existing session
- `citations` (for assistant messages) must reference existing book content
- `confidence_score` must be between 0 and 1

## State Transitions

### Module States
- `draft` → `review`: When content is ready for review
- `review` → `published`: When content passes review and is ready for public access
- `published` → `review`: When updates are needed to published content

### Content States
- `draft` → `review`: When content is ready for validation
- `review` → `published`: When content passes all validation checks
- `published` → `review`: When content needs updates
- `review` → `draft`: When major changes are needed

### Chat Session States
- `active` → `completed`: When conversation is concluded by user
- `active` → `expired`: When session times out after inactivity