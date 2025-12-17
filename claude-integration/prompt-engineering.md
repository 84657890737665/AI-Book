
---

### File 2: `prompt-engineering.md`

```markdown
---
title: Prompt Engineering for Claude
sidebar_position: 2
---

# Prompt Engineering for Claude

## Understanding Claude's Capabilities

### Claude 3 Model Family
| Model | Best For | Context Window | Strengths |
|-------|----------|----------------|-----------|
| **Claude 3 Opus** | Complex reasoning | 200K tokens | Most capable, expensive |
| **Claude 3 Sonnet** | Balanced tasks | 200K tokens | Best value, versatile |
| **Claude 3 Haiku** | Fast responses | 200K tokens | Fastest, cost-effective |

## Core Prompt Engineering Principles

### 1. The Role Pattern
```typescript
// Define Claude's role explicitly
const systemPrompt = `You are an expert TypeScript developer and technical writer.
Your task is to generate clean, maintainable code and documentation.
Follow these guidelines:
1. Write production-ready code
2. Include TypeScript types
3. Add JSDoc comments
4. Consider edge cases
5. Make it accessible`;

// Usage
await claude.sendMessage(userPrompt, systemPrompt);