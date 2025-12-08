
---

### File 4: `best-practices.md`

```markdown
---
title: Best Practices
sidebar_position: 4
---

# SpecifyPlus Best Practices

## AI Component Generation

### 1. Prompt Engineering Guidelines

**Do: Be Specific and Structured**
```typescript
// GOOD: Clear, structured prompt
const goodPrompt = `
Generate a React Button component with:

COMPONENT REQUIREMENTS:
1. Name: PrimaryButton
2. Variants: primary, secondary, danger
3. Sizes: sm, md, lg
4. States: loading, disabled
5. Accessibility: full ARIA support

TECHNICAL REQUIREMENTS:
- TypeScript with strict types
- CSS Modules for styling
- React Testing Library tests
- Storybook stories

DESIGN SPECS:
- Border radius: 8px
- Padding: vertical 12px, horizontal 24px
- Font: Inter, 16px
`;

// BAD: Vague prompt
const badPrompt = "Make a button";