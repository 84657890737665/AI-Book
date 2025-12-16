# Physical AI & Humanoid Robotics Textbook

Welcome to the Physical AI & Humanoid Robotics: From Digital Brain to Physical Body textbook project. This Docusaurus-based textbook provides a comprehensive guide to building intelligent systems that operate in physical space.

## Overview

This textbook bridges the gap between artificial intelligence and physical robotics, focusing on:
- ROS 2 fundamentals for robotic communication
- Simulation environments for safe development
- AI perception systems using Visual SLAM
- Vision-Language-Action convergence for natural interaction

## Getting Started

### Prerequisites

- Node.js (version 18 or higher)
- npm or yarn package manager

### Installation

1. Clone the repository:
```bash
git clone https://github.com/your-username/physical-ai-textbook.git
```

2. Navigate to the project directory:
```bash
cd physical-ai-textbook
```

3. Install dependencies:
```bash
npm install
```

### Local Development

```bash
npm start
```

This command starts a local development server and opens the website in your browser. Most changes are reflected live without restarting the server.

### Build

```bash
npm run build
```

This command generates static content into the `build` directory and can be served using any static content hosting service.

## Project Structure

```
physical-ai-textbook/
├── docs/                 # Textbook content
│   ├── overview/         # Quarter overview content
│   ├── module-1/         # ROS 2 fundamentals
│   ├── module-2/         # Simulation environments  
│   ├── module-3/         # AI perception
│   ├── module-4/         # VLA convergence
│   └── resources/        # Additional materials
├── src/                  # Docusaurus source files
│   └── pages/            # Custom pages
├── static/               # Static assets
│   ├── img/              # Images
│   └── videos/           # Video content
├── docusaurus.config.js  # Docusaurus configuration
├── package.json          # Package dependencies
├── sidebars.js           # Documentation sidebar configuration
└── README.md             # This file
```

## Contributing

We welcome contributions to improve this textbook! Please feel free to submit pull requests or open issues to suggest improvements.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

This textbook leverages the power of Docusaurus for documentation and learning. We acknowledge the open-source community that makes tools like this possible.