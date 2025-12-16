---
id: 1
title: Update project constitution
stage: constitution
date_iso: 2025-12-13
surface: agent
model: Qwen Code
feature: none
branch: main
user: A-ONE LAPTOP
command: /sp.constitution
labels: ["constitution", "setup", "governance"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - ".specify/memory/constitution.md"
tests_yaml: []
prompt_text: |
  $ARGUMENTS

  /sp.constitution Integrated RAG Chatbot Embedded in Published Book
  Target audience:
  Portfolio reviewers, potential employers in AI/full-stack development, and technical recruiters evaluating advanced RAG and agentic AI projects
  Focus:

  Accurate, grounded responses to questions about the book's content using Retrieval-Augmented Generation
  Support for user-selected text highlighting for contextual queries (ad-hoc RAG on selected passages)
  Clean, production-ready architecture that stands out as a polished, innovative portfolio piece
  Emphasis on cost-effective, high-performance stack with Cohere for embeddings/generation and Qdrant/Neon for storage

  Success criteria:

  Chatbot accurately answers 95%+ of test questions about the book content with verifiable citations from retrieved chunks
  Supports two modes: full-book queries and queries restricted to user-selected/highlighted text
  Provides fine-grained citations linking responses to source passages in the book
  Fully embedded/interactive widget in a published digital book (e.g., via HTML/JS embed, Streamlit, Gradio, or static site)
  Clean, well-documented codebase with tests, README, and deployment instructions
  Demonstrates spec-driven development principles (clear specs, plans, ADRs) for maintainability and professionalism
  Live demo accessible online; portfolio showcase highlights innovation (e.g., ad-hoc selection feature)

  Constraints:

  Backend: FastAPI for API endpoints
  Database: Neon Serverless Postgres (with pgvector extension for hybrid storage if needed)
  Vector store: Qdrant Cloud Free Tier
  LLM/Embeddings: Cohere API (Command R/R+ for generation, Embed for vectors); no OpenAI usage
  Development process: Use SpecifyPlus kit (/sp commands) for spec-driven workflow and Qwen CLI (Qwen-Code) as primary AI coding agent
  Book content: Assume provided as PDF/text/Markdown; implement ingestion pipeline for chunking and embedding
  Frontend embedding: Interactive chat interface embeddable in a static/digital book page (e.g., iframe, JS widget)
  Free-tier friendly: Stay within Qdrant free limits and reasonable Cohere usage
  Language: Python primary; clean modern code (type hints, async where appropriate)

  Not building:

  Full user authentication or multi-user session management
  Advanced multi-modal support (e.g., images in book unless text-only)
  Custom fine-tuning of models
  Mobile app or native desktop client
  Comprehensive book publishing platform (focus on chatbot embed only)
  Real-time collaboration or editing features
  Paid-tier scaling or enterprise monitoring
response_text: |
  The constitution file has been successfully updated for the RAG Chatbot Embedded in Published Book project. The new constitution includes six core principles:
  1. Accurate, Grounded Responses
  2. User-Selected Text Highlighting Support
  3. Clean, Production-Ready Architecture
  4. Cost-Effective, High Performance
  5. Spec-Driven Development
  6. Verifiable Citations

  It also defines the technology stack requirements, development workflow, and governance guidelines specific to this project.

  Version: 1.0.0 | Ratified: 2025-01-01 | Last Amended: 2025-12-13

  A sync impact report was included as an HTML comment at the top of the constitution file.
outcome: |
  Successfully created and populated the project constitution for the RAG Chatbot project.
  The new constitution reflects the specific requirements, constraints, and goals of the project.
  Dependent templates were checked for consistency.
  A Prompt History Record was created as required.
evaluation: |
  All required tasks were completed:
  - Loaded and analyzed the existing constitution template
  - Identified all placeholder tokens
  - Collected/derived values from the provided project information
  - Drafted updated constitution with concrete values
  - Performed consistency checks on dependent templates
  - Created sync impact report
  - Validated the final constitution document
  - Wrote the completed constitution to the target file
  - Provided summary to the user
  - Created this Prompt History Record
---

# [PROJECT_NAME] Constitution
<!-- Example: Spec Constitution, TaskFlow Constitution, etc. -->

## Core Principles

### [PRINCIPLE_1_NAME]
<!-- Example: I. Library-First -->
[PRINCIPLE_1_DESCRIPTION]
<!-- Example: Every feature starts as a standalone library; Libraries must be self-contained, independently testable, documented; Clear purpose required - no organizational-only libraries -->

### [PRINCIPLE_2_NAME]
<!-- Example: II. CLI Interface -->
[PRINCIPLE_2_DESCRIPTION]
<!-- Example: Every library exposes functionality via CLI; Text in/out protocol: stdin/args → stdout, errors → stderr; Support JSON + human-readable formats -->

### [PRINCIPLE_3_NAME]
<!-- Example: III. Test-First (NON-NEGOTIABLE) -->
[PRINCIPLE_3_DESCRIPTION]
<!-- Example: TDD mandatory: Tests written → User approved → Tests fail → Then implement; Red-Green-Refactor cycle strictly enforced -->

### [PRINCIPLE_4_NAME]
<!-- Example: IV. Integration Testing -->
[PRINCIPLE_4_DESCRIPTION]
<!-- Example: Focus areas requiring integration tests: New library contract tests, Contract changes, Inter-service communication, Shared schemas -->

### [PRINCIPLE_5_NAME]
<!-- Example: V. Observability, VI. Versioning & Breaking Changes, VII. Simplicity -->
[PRINCIPLE_5_DESCRIPTION]
<!-- Example: Text I/O ensures debuggability; Structured logging required; Or: MAJOR.MINOR.BUILD format; Or: Start simple, YAGNI principles -->

### [PRINCIPLE_6_NAME]


[PRINCIPLE__DESCRIPTION]

## [SECTION_2_NAME]
<!-- Example: Additional Constraints, Security Requirements, Performance Standards, etc. -->

[SECTION_2_CONTENT]
<!-- Example: Technology stack requirements, compliance standards, deployment policies, etc. -->

## [SECTION_3_NAME]
<!-- Example: Development Workflow, Review Process, Quality Gates, etc. -->

[SECTION_3_CONTENT]
<!-- Example: Code review requirements, testing gates, deployment approval process, etc. -->

## Governance
<!-- Example: Constitution supersedes all other practices; Amendments require documentation, approval, migration plan -->

[GOVERNANCE_RULES]
<!-- Example: All PRs/reviews must verify compliance; Complexity must be justified; Use [GUIDANCE_FILE] for runtime development guidance -->

**Version**: [CONSTITUTION_VERSION] | **Ratified**: [RATIFICATION_DATE] | **Last Amended**: [LAST_AMENDED_DATE]
<!-- Example: Version: 2.1.1 | Ratified: 2025-06-13 | Last Amended: 2025-07-16 -->