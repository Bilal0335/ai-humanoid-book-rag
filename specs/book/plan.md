# Implementation Plan: AI/Spec-driven Book Creation with Docusaurus

**Branch**: `book-main` | **Date**: 2025-12-11 | **Spec**: [specs/book/spec.md](spec.md)

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Create a comprehensive system for AI/Spec-driven book creation using Docusaurus that follows the project constitution principles. The system will include architecture for content structure, module flow, cross-references, MDX patterns, quality validation, and integration with the RAG chatbot. The plan encompasses five modules plus capstone project with consistent structure and educational clarity for senior students.

## Technical Context

**Language/Version**: JavaScript/TypeScript for Docusaurus, Python 3.10+ for backend services, Markdown/MDX for content, ROS 2 Humble for robotics framework
**Primary Dependencies**: Docusaurus 2.x, Node.js 18+, ROS 2 Humble, FastAPI 0.104+, OpenAI SDK, Qdrant 1.6+, Neon Postgres, NVIDIA Isaac Sim, Gazebo, Unity
**Storage**: Git-based content management, Qdrant Cloud for vector embeddings, Neon Serverless Postgres for metadata, Docusaurus static build
**Testing**: pytest for backend, manual validation for robotics workflows, content review processes, Docusaurus build validation, automated link checking
**Target Platform**: Ubuntu 22.04 (local/WSL2), GitHub Pages for frontend, cloud deployment for backend services
**Project Type**: Documentation + Web application + Robotics workflows
**Performance Goals**: 15+ FPS for simulations, <2s response time for RAG chatbot, 95% uptime for backend services, <5s Docusaurus build time, 90%+ success rate for all code examples
**Constraints**: <15 FPS acceptable for learning, <5s response time for RAG acceptable, offline-capable content with online RAG integration, reproducible workflows across environments
**Scale/Scope**: 150-200 Docusaurus pages (MDX), 5 modules with 10-20 sections each, 1000+ students potential reach, multi-framework integration (ROS 2, Gazebo, Unity, Isaac Sim, Nav2, VSLAM)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on the project constitution, this implementation plan must:
- ✅ Validate all technical content against primary robotics/AI documentation (ROS 2, Gazebo, Unity, Isaac, OpenAI SDKs)
- ✅ Achieve conceptual clarity for senior students in AI, robotics, and software engineering
- ✅ Ensure reproducibility with step-by-step workflows and validation criteria
- ✅ Maintain interoperability between book content, code samples, and RAG chatbot
- ✅ Maintain academic rigor with 50%+ references to official documentation and research papers
- ✅ Follow the prescribed four-module structure plus capstone project
- ✅ Use Docusaurus MDX format with clean component structure
- ✅ Ensure code samples are runnable on modern Linux systems
- ✅ Support both whole-book Q&A and selected-text-only Q&A for RAG chatbot
- ✅ Meet content length requirements equivalent to 150-200 Docusaurus pages

## Project Structure

### Documentation (this feature)

```text
specs/book/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
docs/
├── src/
│   ├── components/
│   │   ├── RAGChatbot/
│   │   │   ├── ChatInterface.jsx
│   │   │   └── CitationRenderer.jsx
│   │   ├── diagrams/
│   │   ├── modules/
│   │   │   ├── ros2/
│   │   │   ├── simulation/
│   │   │   ├── navigation/
│   │   │   ├── vla/
│   │   │   └── capstone/
│   │   └── common/
│   ├── pages/
│   └── theme/
├── static/
│   ├── img/
│   ├── models/          # URDF files and 3D assets
│   └── media/           # Videos, audio, and other assets
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

backend/
├── src/
│   ├── main.py
│   ├── models/
│   │   ├── content.py
│   │   ├── embedding.py
│   │   └── chat.py
│   ├── services/
│   │   ├── rag_service.py
│   │   ├── content_service.py
│   │   └── citation_service.py
│   ├── routers/
│   │   ├── chat.py
│   │   ├── content.py
│   │   └── health.py
│   └── utils/
│       ├── content_parser.py
│       └── citation_formatter.py
├── requirements.txt
├── Dockerfile
└── tests/
    ├── test_rag.py
    ├── test_content.py
    └── test_citation.py

tests/
├── integration/
│   ├── test_book_content.py
│   └── test_rag_integration.py
└── validation/
    ├── test_ros2_examples.py
    ├── test_simulation_workflows.py
    └── test_vla_workflows.py

scripts/
├── setup/
│   ├── install_ros2.sh
│   ├── setup_isaac.sh
│   └── setup_unity.sh
├── validation/
│   ├── test_ros2_nodes.sh
│   ├── test_gazebo_sim.sh
│   └── test_isaac_sim.sh
└── deployment/
    ├── deploy_docs.sh
    └── deploy_backend.sh
```

**Structure Decision**: Web application structure with separate documentation frontend (Docusaurus) and backend services (FastAPI) to support the RAG chatbot functionality while maintaining content in a single, organized documentation structure.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Multi-repo structure (docs + backend) | RAG chatbot requires separate backend services for embeddings and retrieval | Single documentation approach insufficient for interactive chatbot functionality |
| Complex deployment (GitHub Pages + cloud backend) | Educational value of showing full AI-robotics integration stack | Static documentation only insufficient for interactive learning experience |
| Multiple simulation environments | Students need to understand cross-platform compatibility and model reuse | Single simulation environment insufficient for comprehensive robotics education |
| Advanced GPU requirements | Isaac Sim and Unity require significant GPU power for photorealistic rendering | Lower-end hardware insufficient for learning objectives |