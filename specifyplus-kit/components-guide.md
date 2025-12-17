
---

### File 3: `components-guide.md`

```markdown
---
title: Components Guide
sidebar_position: 3
---

# SpecifyPlus Components Guide

## Component Generation Workflow

### 1. Basic Component Generation
```bash
# Generate a simple button component
npx specifyplus generate button --ai

# Generate with specific props
npx specifyplus generate button \
  --props="variant:primary|secondary|danger,size:sm|md|lg" \
  --framework=react \
  --typescript

# Generate with custom styles
npx specifyplus generate card \
  --styles="shadow,rounded,border" \
  --styling=tailwind