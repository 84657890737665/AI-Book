---
id: 1
title: AI Research Paper Constitution
stage: constitution
date: 2025-12-11
surface: agent
model: gpt-4o
feature: none
branch: main
user: user
command: /sp.constitution
labels: [constitution, research-paper, ai]
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

```
/sp.constitution 

Project: Research paper on AI-native software development

Core principles:
- Accuracy through primary source verification
- Clarity for academic audience (computer science background)
- Reproducibility (all claims cited and traceable)
- Rigor (peer-reviewed sources preferred)

Key standards:
- All factual claims must be traceable to sources
- Citation format: APA style
- Source types: minimum 50% peer-reviewed articles
- Plagiarism check: 0% tolerance before submission
- Writing clarity: Flesch-Kincaid grade 10-12

Constraints:
- Word count: 5,000-7,000 words
- Minimum 15 sources
- Format: PDF with embedded citations

Success criteria:
- All claims verified against sources
- Zero plagiarism detected
- Passes fact-checking review
```

## Response snapshot

I've successfully updated the project constitution for your research paper on AI-native software development. Here's the summary:

1. **New version**: 1.0.0 (initial version)
   - **Bump rationale**: This is the initial version of the constitution for the research paper project, establishing core principles for academic research work

2. **Files flagged for manual follow-up**:
   - `.specify/templates/plan-template.md` - contains "Constitution Check" that may need to reference the new principles
   - `.specify/templates/spec-template.md` - may need to align scope/requirements with research paper standards
   - `.specify/templates/tasks-template.md` - may need to reflect research paper task types rather than software development tasks
   - `.claude/commands/*.md` - should be reviewed for any outdated references to generic software development vs. academic research

3. **Suggested commit message**: `docs: add constitution for AI-native software development research paper v1.0.0`

The constitution now contains six core principles specifically tailored for academic research:
1. Accuracy through Primary Source Verification
2. Clarity for Academic Audience
3. Reproducibility
4. Rigor through Peer-Reviewed Sources
5. Fact-Traceability (NON-NEGOTIABLE)
6. Zero-Tolerance for Academic Dishonesty

All additional standards, constraints, and success criteria you specified have been incorporated into the appropriate sections.

The constitution file has been updated at `.specify/memory/constitution.md` with a Sync Impact Report at the top documenting all changes made.

## Outcome

- ✅ Impact: Created new constitution for AI research paper project with 6 core principles
- 🧪 Tests: No tests required for constitution document
- 📁 Files: Updated .specify/memory/constitution.md with research-specific principles
- 🔁 Next prompts: Could create spec/plan/tasks for the research paper using new constitution
- 🧠 Reflection: Constitution properly adapted for academic research context instead of software development

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Use new constitution to guide research paper development