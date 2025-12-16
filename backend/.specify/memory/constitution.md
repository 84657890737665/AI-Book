<!-- 
SYNC IMPACT REPORT
Version change: 1.0.0 → 1.1.0
Added sections: Testing standards, API usage standards, Database schema standards, Vector store standards, Error handling standards, Additional constraints
Removed sections: None
Modified principles: 
  I. Accuracy and relevance in responses through robust retrieval and generation mechanisms (was: Accurate, Grounded Responses)
  II. User-centric design for seamless integration (was: User-Selected Text Highlighting Support) 
  III. Efficiency and scalability using serverless and cloud-free tiers (was: Clean, Production-Ready Architecture)
  IV. Security and privacy compliance (was: Cost-Effective, High Performance)
  V. Innovation by adapting Cohere API (was: Spec-Driven Development)
  VI. Modularity and documentation (new sixth principle)
Templates requiring updates: 
✅ .specify/templates/plan-template.md - Constitution Check section updated
✅ .specify/templates/spec-template.md - Requirements section updated
✅ .specify/templates/tasks-template.md - Task categories updated
⚠️  .specify/commands/sp.constitution.toml - May need review for outdated references
Follow-up TODOs: None
-->
# Integrated RAG Chatbot Development Constitution

## Core Principles

### I. Accuracy and Relevance
The system MUST provide accurate and relevant responses through robust retrieval and generation mechanisms using Cohere API.

### II. User-Centric Design
The system MUST provide seamless integration and intuitive interaction within the book, including responses based solely on user-selected text.

### III. Efficiency and Scalability
The system MUST leverage serverless and cloud-free tiers efficiently to maintain scalability and cost-effectiveness.

### IV. Security and Privacy
The system MUST comply with security and privacy requirements, especially for user-selected text handling.

### V. Innovation
The system MUST demonstrate innovation by adapting Cohere API for enhanced natural language understanding.

### VI. Modularity and Documentation
All code MUST be modular, well-documented, and follow PEP 8 style guidelines.

## Standards and Requirements

### Testing Standards
Integration testing: 100% coverage for core functionalities (retrieval, generation, user text selection)

### API Usage Standards
API usage: Optimize Cohere API calls for cost-efficiency and performance.

### Database Standards
Database schema: Normalized and indexed for fast queries in Neon Postgres.

### Vector Store Standards
Vector store: Efficient embedding and similarity search in Qdrant.

### Error Handling Standards
Error handling: Graceful degradation and logging for all components.

## Constraints

- API switch: Exclusively use Cohere API; no OpenAI dependencies
- Tools: Development must incorporate Speckit Plus and Qwen CLI throughout
- Budget: Rely on free tiers (Qdrant Cloud Free, Neon Serverless) with minimal external costs
- Timeline: Modular milestones for prototyping, testing, and embedding
- Compatibility: Ensure chatbot embeds seamlessly in common book formats (e.g., PDF, EPUB)

## Success Criteria

- Chatbot accurately answers 95%+ of book-related queries in blind tests
- Handles user-selected text isolation without context leakage
- Zero critical security vulnerabilities detected in code review
- Full deployment and embedding in a sample book with positive user feedback
- Comprehensive documentation and reproducible setup via Git repository

## Governance

All implementation must follow the specified constraints and requirements.
All PRs/reviews must verify compliance with the defined success criteria.
Architecture decisions must be documented as ADRs when significant.
The solution should demonstrate innovation, particularly in the ad-hoc selection feature.

**Version**: 1.1.0 | **Ratified**: 2025-01-01 | **Last Amended**: 2025-12-13