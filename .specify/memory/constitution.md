<!--
Sync Impact Report:
- Version change: N/A -> 1.0.0 (initial constitution)
- Modified principles: N/A (new document)
- Added sections: All sections (new document)
- Removed sections: None
- Templates requiring updates: ✅ Updated
- Follow-up TODOs: None
-->

# AI-Driven Technical Book with Embedded RAG Chatbot Constitution

## Core Principles

### I. Spec-Driven and Reproducible Execution
Every change follows a documented specification-first approach; All processes must be scriptable, repeatable, and verifiable; Every execution produces consistent, predictable outcomes regardless of environment.

### II. Book Content as Single Source of Truth
Book content serves as the authoritative foundation for all derived artifacts; All documentation, examples, and RAG indexing originates from the primary book content; Content changes must be reflected consistently across all downstream systems.

### III. Developer-Focused Writing (NON-NEGOTIABLE)
All content must be clear, actionable, and immediately applicable to developers; Technical explanations include runnable code examples with expected outputs; Concepts are demonstrated with practical, real-world implementations.

### IV. Strict Grounding with Zero Hallucinations
The RAG chatbot responds only to information explicitly present in book content or user-selected text; Responses must cite specific sources within the book content; Out-of-scope questions are explicitly refused with clear explanations.

### V. Free-Tier Service Constraint
All infrastructure components must operate within free-tier service limitations; Architecture and implementation choices prioritize cost-effective solutions; No paid services may be required for basic functionality.

### VI. Minimal Viable Architecture
Start with simplest possible implementation that meets requirements; Additional complexity must be justified by clear functional or operational benefits; YAGNI (You Aren't Gonna Need It) principle applies to features and infrastructure.

## Technical Standards

### Technology Stack Requirements
- Book built with Docusaurus for static site generation and deployment
- RAG backend implemented with FastAPI for scalability and performance
- Vector storage uses Qdrant Cloud Free Tier for document similarity search
- Database persistence with Neon Postgres for metadata and user interactions
- Chatbot powered by OpenAI Agents/ChatKit for intelligent responses

### Deployment and Hosting Policy
- Static book content deployed to GitHub Pages for accessibility and reliability
- Backend API deployed to platforms supporting free tier usage (Railway, Render, etc.)
- Deployment process must be scriptable and reproducible from README instructions
- All deployment configurations stored in version control with clear documentation

## Development Workflow

### Content Creation and Review Process
- All book content follows structured format with clear examples and explanations
- Code examples must be tested and verified as runnable before inclusion
- Content changes undergo peer review focusing on clarity, accuracy, and completeness
- RAG indexing pipeline must update automatically when content changes

### Quality Assurance Requirements
- All code examples compile and run as documented
- RAG chatbot responses are validated against ground truth content
- End-to-end testing covers content-to-response pipeline
- Performance benchmarks ensure acceptable response times within free-tier constraints

## Governance

This constitution governs all development decisions for the AI-Driven Technical Book with Embedded RAG Chatbot project. All contributors must comply with these principles. Amendments require explicit documentation of changes, justification for deviations, and approval from project maintainers. All pull requests must demonstrate compliance with these principles through tests, documentation, and code review.

**Version**: 1.0.0 | **Ratified**: 2025-12-19 | **Last Amended**: 2025-12-19
