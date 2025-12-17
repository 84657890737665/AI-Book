
---

### File 2: `configuration.md`

```markdown
---
title: SpecifyPlus Configuration
sidebar_position: 2
---

# SpecifyPlus Configuration Guide

## Configuration Files Structure

### 1. Main Configuration
```javascript
// specifyplus.config.js
module.exports = {
  // Project metadata
  project: {
    name: 'AI Native Book',
    version: '1.0.0',
    description: 'AI-driven development documentation'
  },
  
  // Design system configuration
  designSystem: {
    prefix: 'ai',
    theme: 'light',
    scale: '1.2'
  },
  
  // Framework settings
  framework: {
    name: 'react',
    version: '18',
    typescript: true,
    styling: 'tailwind' // or 'css-modules', 'styled-components'
  },
  
  // AI settings (detailed in next section)
  ai: {
    enabled: true,
    providers: ['claude', 'openai'],
    autoApprove: false
  },
  
  // Output configuration
  output: {
    components: './src/components/generated',
    tokens: './src/styles/tokens',
    docs: './docs/components',
    tests: './src/components/__tests__'
  },
  
  // Quality checks
  quality: {
    eslint: true,
    prettier: true,
    tests: true,
    accessibility: true
  }
};