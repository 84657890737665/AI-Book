
---

### File 4: `tooling-overview.md`

```markdown
---
title: AI Development Tooling
sidebar_position: 4
---

# AI Development Tooling Overview

## Essential Tool Stack

### Core Development
| Tool | Purpose | AI Integration |
|------|---------|---------------|
| **VS Code** | Code editor | GitHub Copilot, Codeium |
| **Node.js** | Runtime | AI SDKs, Model serving |
| **Git** | Version control | AI commit messages, PR reviews |

### AI-Specific Tools
| Tool | Type | Best For |
|------|------|----------|
| **SpecifyPlus Kit** | Component Library | UI generation |
| **Claude AI** | Language Model | Code generation, docs |
| **GitHub Copilot** | AI Pair Programmer | Inline code suggestions |
| **Cursor** | AI-First Editor | Complete AI workflow |

## VS Code Setup for AI Development

### Essential Extensions
```json
// .vscode/extensions.json
{
  "recommendations": [
    // AI Extensions
    "GitHub.copilot",
    "GitHub.copilot-chat",
    "codeium.codeium",
    
    // TypeScript & React
    "ms-vscode.vscode-typescript-next",
    "dsznajder.es7-react-js-snippets",
    
    // Documentation
    "davidanson.vscode-markdownlint",
    "yzhang.markdown-all-in-one",
    
    // Testing
    "orta.vscode-jest",
    
    // Git & AI
    "github.vscode-pull-request-github"
  ]
}