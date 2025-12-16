// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
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

module.exports = sidebars;