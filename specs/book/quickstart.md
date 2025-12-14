# Quickstart Guide: AI/Spec-driven Book Creation with Docusaurus

**Date**: 2025-12-11
**Feature**: AI/Spec-driven Book Creation with Docusaurus

## Overview

This guide provides the essential steps to set up the development environment for the Physical AI & Humanoid Robotics book project using Docusaurus and the Spec-Kit Plus + Claude Code workflow. The system enables specification-driven book creation with integrated RAG functionality.

## Prerequisites

### System Requirements
- Operating System: Ubuntu 22.04 LTS (or WSL2 on Windows)
- RAM: 16GB minimum (32GB recommended)
- Storage: 50GB free space
- GPU: NVIDIA GPU with 8GB+ VRAM (for Isaac Sim and Unity simulations)
- Node.js: 18+ and npm/yarn
- Python: 3.10 or higher
- Git

### Required Software
- Git
- Node.js 18+ and npm/yarn
- Python 3.10+
- ROS 2 Humble Hawksbill
- Docker (for containerized backend services)

## Environment Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd my-research-paper
```

### 2. Set Up Docusaurus Development Environment
```bash
# Navigate to docs directory
cd docs

# Install dependencies
npm install

# Verify Docusaurus installation
npx docusaurus --version
```

### 3. Set Up Backend Services
```bash
# Navigate to backend directory
cd ../backend

# Create virtual environment
python3 -m venv book_env
source book_env/bin/activate

# Install Python dependencies
pip install --upgrade pip
pip install -r requirements.txt
```

### 4. Configure Environment Variables
```bash
# Create .env file in backend directory
cd ../backend
cp .env.example .env

# Edit .env with your specific configurations:
# OPENAI_API_KEY=your_openai_api_key
# QDRANT_URL=your_qdrant_url
# QDRANT_API_KEY=your_qdrant_api_key
# NEON_DATABASE_URL=your_neon_database_url
```

### 5. Install ROS 2 Humble (if not already installed)
```bash
# Add ROS 2 repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-ros-base
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2 environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Project Structure Navigation

### Book Content Organization
```
docs/
├── src/
│   ├── components/
│   │   ├── RAGChatbot/
│   │   │   ├── ChatInterface.jsx
│   │   │   └── CitationRenderer.jsx
│   │   ├── diagrams/
│   │   └── modules/
│   │       ├── ros2/
│   │       ├── simulation/
│   │       ├── navigation/
│   │       ├── vla/
│   │       └── capstone/
│   ├── pages/
│   └── theme/
├── static/
│   ├── img/
│   └── models/          # URDF files and 3D assets
├── modules/
│   ├── module-1-ros2/
│   │   ├── foundations/
│   │   ├── communication/
│   │   └── integration/
│   ├── module-2-simulation/
│   │   ├── gazebo/
│   │   ├── unity/
│   │   └── isaac/
│   ├── module-3-navigation/
│   │   ├── vslam/
│   │   ├── nav2/
│   │   └── sensor-fusion/
│   ├── module-4-vla/
│   │   ├── vision-language/
│   │   ├── action-planning/
│   │   └── embodied-ai/
│   └── module-5-capstone/
│       └── integration/
├── docusaurus.config.js
├── sidebars.js
└── package.json
```

### Specification-Driven Development Workflow
```
specs/
├── book/                   # Main book specification
│   ├── spec.md            # Feature specification
│   ├── plan.md            # Implementation plan
│   ├── research.md        # Research findings
│   ├── data-model.md      # Data model
│   ├── quickstart.md      # This guide
│   ├── contracts/         # API contracts
│   └── tasks.md           # Implementation tasks
└── [module-number]-[module-name]/ # Individual module specs
    ├── spec.md
    ├── plan.md
    └── ...
```

## Using the Spec-Kit Plus + Claude Code Workflow

### 1. Create a New Feature Specification
```bash
# Use the /sp.specify command (as implemented in the system)
# This generates a spec.md file based on your feature description
```

### 2. Generate Implementation Plan
```bash
# Use the /sp.plan command to generate plan.md
# This creates the technical architecture and phases
```

### 3. Generate Implementation Tasks
```bash
# Use the /sp.tasks command to generate tasks.md
# This creates detailed implementation steps
```

### 4. Create Content Using Claude Code
- Follow the generated tasks to implement content
- Use Docusaurus MDX format for all content
- Include proper citations and diagrams as specified

## Running the Development Server

### 1. Start Docusaurus Development Server
```bash
cd docs
npm run start
```
The book will be available at http://localhost:3000

### 2. Start Backend Services
```bash
cd backend
source ../book_env/bin/activate
uvicorn src.main:app --reload --port 8000
```

### 3. Initialize the RAG System
```bash
# In a separate terminal, with the Python environment activated
cd backend
source ../book_env/bin/activate
python -c "from src.services.content_service import initialize_rag; initialize_rag()"
```

## Content Development Workflow

### 1. Adding New Content
1. Create new MDX files in the appropriate module directory under `docs/modules/`
2. Update `docs/sidebars.js` to include the new content in the navigation
3. Add content to the RAG system by running the content ingestion script

### 2. Content Ingestion for RAG
```bash
cd backend
source ../book_env/bin/activate
python -c "from src.services.content_service import ingest_book_content; ingest_book_content()"
```

### 3. Testing Content
```bash
# Test Docusaurus build
cd docs
npm run build

# Test backend functionality
cd backend
source ../book_env/bin/activate
python -m pytest tests/
```

## Quality Validation

### 1. Content Accuracy Validation
- Verify all code examples run successfully in the target environment
- Check that all citations link to valid sources
- Confirm technical statements align with official documentation
- Validate that all examples are reproducible in Ubuntu 22.04/WSL2

### 2. Build Integrity Validation
```bash
# Check Docusaurus build
cd docs
npm run build

# Validate internal linking
npm run serve

# Check for broken links
npx @docusaurus/plugin-client-redirects
```

### 3. Constitution Compliance Validation
- Ensure all content meets the academic rigor requirements (50%+ official documentation)
- Verify content adheres to target audience level (senior undergraduate/graduate)
- Confirm all code examples are runnable on specified platforms
- Validate RAG integration follows strict citation mode requirements

## Deployment

### 1. Build Documentation
```bash
cd docs
npm run build
```

### 2. Deploy to GitHub Pages
```bash
# Docusaurus provides a built-in command for GitHub Pages deployment
GIT_USER=<your-github-username> npm run deploy
```

### 3. Deploy Backend Services
The backend services should be deployed to a cloud provider with GPU support for Isaac Sim and Unity simulations.

## Troubleshooting

### Common Issues

1. **Docusaurus Build Issues**:
   - Check for MDX syntax errors in new content
   - Verify all internal links are properly formatted
   - Ensure all required npm packages are installed

2. **ROS 2 Installation Issues**:
   - Ensure correct Ubuntu version (22.04)
   - Check for conflicting ROS installations
   - Verify network connectivity for package installation

3. **RAG Service Issues**:
   - Verify API keys are correctly configured
   - Check connectivity to Qdrant and Neon services
   - Ensure content has been properly ingested

### Useful Commands

```bash
# Check Docusaurus installation
npx docusaurus --version

# Check Python environment
python --version
pip list | grep -i ros

# Check Docker services
docker ps

# Check Node.js and npm
node --version
npm --version
```

## Next Steps

1. Begin developing content for Module 1 (ROS 2 Nervous System)
2. Implement the unified humanoid robot model in URDF format
3. Set up the RAG system with initial book content
4. Create simulation environments for Gazebo and Isaac Sim
5. Develop the integration between Docusaurus frontend and RAG backend