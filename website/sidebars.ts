import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
    },
    {
      type: 'doc',
      id: 'textbook-overview',
    },
    {
      type: 'category',
      label: 'Module 1: Robotic Nervous System (ROS 2)',
      items: [
        'module-1/ros2-intro',
        'module-1/nodes-topics-services',
        'module-1/rclpy-python-agents',
        'module-1/urdf-integration',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation Environments',
      items: [
        'module-2/simulation-intro',
        'module-2/gazebo-fundamentals',
        'module-2/isaac-sim',
        'module-2/unity-environments',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: AI Perception',
      items: [
        'module-3/perception-intro',
        'module-3/visual-slam',
        'module-3/computer-vision',
        'module-3/sensor-fusion',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action Convergence',
      items: [
        'module-4/vla-intro',
        'module-4/whisper-integration',
        'module-4/llm-planning',
        'module-4/voice-to-action-pipeline',
      ],
    },
    {
      type: 'category',
      label: 'Module 5: Physical Implementation',
      items: [
        'module-5/physical-intro',
        'module-5/jetson-platforms',
        'module-5/hardware-integration',
        'module-5/real-robot-operation',
      ],
    },
  ],

  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

export default sidebars;