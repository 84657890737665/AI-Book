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
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Introduction',
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
      ],
    },
    {
      type: 'category',
      label: 'Module 3: AI Perception',
      items: [
        'module-3/perception-intro',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action Convergence',
      items: [
        'module-4/vla-intro',
      ],
    },
    {
      type: 'category',
      label: 'Module 5: Physical Implementation',
      items: [
        'module-5/physical-intro',
      ],
    },
  ],
};

export default sidebars;
