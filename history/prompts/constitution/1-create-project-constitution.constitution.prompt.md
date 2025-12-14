---
id: 1
title: Create Project Constitution
stage: constitution
date: 2025-12-11
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.constitution
labels: [constitution, governance, project-setup]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

```text
Project: Complete book project on Physical AI & Humanoid Robotics with integrated RAG chatbot

Core principles:
- Technical accuracy validated through primary robotics/AI documentation (ROS 2, Gazebo, Unity, Isaac, OpenAI SDKs)
- Conceptual clarity for senior students in AI, robotics, and software engineering
- Reproducibility: all workflows, code samples, and architectures must be replicable by following step-by-step instructions
- Interoperability: book content, code, and RAG chatbot must function together without conflict
- Rigor: definitions, robotics concepts, AI usage, and simulation workflows must be validated against official sources

Key standards:
- All technical statements must reference authoritative documentation (ROS 2, Gazebo, Unity, Isaac Sim, OpenAI Agents/ChatKit, Qdrant, Neon DB)
- Writing format: Docusaurus MDX with clean component structure
- Code correctness: all code samples and commands runnable on modern Linux systems
- Citation style: inline reference links to official docs + academic citations where needed
- Minimum 50% of conceptual explanations must reference robotics or AI research papers or official documentation
- Quality benchmark: explanations must reach senior undergraduate/early graduate clarity level
- Accessibility: diagrams or architecture sketches must be readable and reproducible

Constraints:
- Output format: Docusaurus book deployed on GitHub Pages
- Mandatory integration: embedded RAG chatbot using OpenAI Agents/ChatKit, FastAPI backend, Qdrant Cloud vector DB, and Neon Serverless Postgres
- RAG must support:
  • whole-book Q&A
  • user-selected-content Q&A
- Book scope: 4 modules + capstone project
- Minimum content length: equivalent of 150–200 Docusaurus pages (MDX)
- All workflows must include: prerequisites, environment setup, architecture diagram, step-by-step instructions, and testing criteria

Project Structure (Required):
- Module 1: Robotic Nervous System (ROS 2)
- Module 2: Digital Twin (Gazebo & Unity)
- Module 3: AI-Robot Brain (NVIDIA Isaac™)
- Module 4: Vision-Language-Action (VLA)
- Capstone: Autonomous Humanoid Robot

Success criteria:
- Book compiles without errors and deploys to GitHub Pages successfully
- RAG chatbot performs accurate contextual Q&A based on book content and selected text
- All robotics simulations (ROS 2, Gazebo, Isaac) described in the book are reproducible
- Code samples validated and tested
- Zero plagiarism and all sources traceable
- Book provides functional, end-to-end guidance from theory → simulation → AI → autonomous humanoid robot behavior
```

## Response snapshot

Created a comprehensive project constitution for the Physical AI & Humanoid Robotics book project. The constitution includes six core principles covering technical accuracy, educational clarity, reproducibility, interoperability, academic rigor, and modularity. It also defines technical standards and constraints, development workflow requirements, and governance procedures. The constitution serves as the governing document for all project activities and ensures adherence to the specified requirements throughout the project lifecycle.

## Outcome

- ✅ Impact: Created foundational governance document for the entire book project
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md with comprehensive project constitution
- 🔁 Next prompts: Proceed with project planning and specification work guided by this constitution
- 🧠 Reflection: The constitution successfully captures all key requirements from the user input and establishes clear principles for the project team to follow.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS - All requirements incorporated into constitution
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin creating project specification based on this constitution