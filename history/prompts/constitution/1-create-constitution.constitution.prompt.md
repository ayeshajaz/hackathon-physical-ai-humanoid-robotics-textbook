---
id: 1
title: Create Constitution
stage: constitution
date: 2025-12-19
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: user
command: /sp.constitution
labels: [constitution, setup, governance]
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
Project:
AI-Driven Technical Book with Embedded RAG Chatbot

Core principles:
- Spec-driven and reproducible execution
- Book content is the single source of truth
- Clear, developer-focused writing
- Strict grounding with zero hallucinations

Key standards:
- Write the book using Claude Code and Spec-Kit Plus
- Build the book with Docusaurus
- Deploy the book to GitHub Pages
- Use clear structure and runnable code examples

RAG requirements:
- Embed the chatbot directly in the book
- Answer questions only from book content or user-selected text
- Explicitly refuse out-of-scope questions
- Tech stack: FastAPI, OpenAI Agents/ChatKit, Neon Postgres, Qdrant Cloud (Free Tier)

Constraints:
- Use free-tier services only
- Indexing must be scriptable and reproducible
- No undocumented steps

Success criteria:
- Book is live on GitHub Pages
- Chatbot provides accurate, grounded responses
- Selected-text question answering works correctly
- Repository runs end-to-end using README instructions
```

## Response snapshot

Created the project constitution file (.specify/memory/constitution.md) with the following sections:

1. Core Principles:
   - Spec-Driven and Reproducible Execution
   - Book Content as Single Source of Truth
   - Developer-Focused Writing (NON-NEGOTIABLE)
   - Strict Grounding with Zero Hallucinations
   - Free-Tier Service Constraint
   - Minimal Viable Architecture

2. Technical Standards:
   - Technology Stack Requirements
   - Deployment and Hosting Policy

3. Development Workflow:
   - Content Creation and Review Process
   - Quality Assurance Requirements

4. Governance section outlining compliance requirements

The constitution reflects all the specified requirements and constraints, establishing a clear governance framework for the project.

## Outcome

- ✅ Impact: Created initial project constitution with all core principles and governance rules
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md with complete constitution
- 🔁 Next prompts: Project setup and initial specification creation
- 🧠 Reflection: Constitution properly captures all specified requirements and constraints

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin with project specification based on constitution