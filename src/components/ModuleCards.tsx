import React from 'react';
import Link from '@docusaurus/Link';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './ModuleCards.module.css'; // Changed to use the new CSS module

type ModuleItem = {
  title: string;
  icon: string;
  description: string;
  path: string; // Path to navigate to when clicked
};

// List of modules/chapters with their paths
const ModuleList: ModuleItem[] = [
  {
    title: 'Robotic Nervous System (ROS 2)',
    icon: '🧠',
    description: 'Fundamentals of ROS 2 - the backbone of robotic systems',
    path: '/docs/module-1/ros2-intro', // First lesson in module 1
  },
  {
    title: 'Simulation Environments',
    icon: '🎮',
    description: 'Simulators like Gazebo, Isaac Sim, and Unity for robotics',
    path: '/docs/module-2/simulation-intro', // Module 2 introduction
  },
  {
    title: 'AI Perception',
    icon: '👁️',
    description: 'Computer vision and sensor processing for environmental awareness',
    path: '/docs/module-3/perception-intro', // Module 3 introduction
  },
  {
    title: 'Vision-Language-Action Convergence',
    icon: '🗣️',
    description: 'How AI integrates vision, language, and action for embodied intelligence',
    path: '/docs/module-4/vla-intro', // Module 4 introduction
  },
  {
    title: 'Physical Implementation',
    icon: '🤖',
    description: 'Moving from simulation to real-world robot operation',
    path: '/docs/module-5/physical-intro', // Module 5 introduction
  },
  {
    title: 'Hardware Integration',
    icon: '🔧',
    description: 'Connecting software to physical components and sensors',
    path: '/docs/hardware/', // Hardware overview
  },
];

function ModuleCard({ title, icon, description, path }: ModuleItem) {
  return (
    <Link to={path} className={clsx(styles.moduleCard, 'padding--lg')}>
      <div className={styles.cardIcon}>
        <span className={styles.emojiIcon}>{icon}</span>
      </div>
      <div className="padding-horiz--md">
        <Heading as="h3" className={styles.cardTitle}>{title}</Heading>
        <p className={styles.cardDescription}>{description}</p>
      </div>
      <div className={styles.cardArrow}>
        <svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
          <line x1="5" y1="12" x2="19" y2="12"></line>
          <polyline points="12 5 19 12 12 19"></polyline>
        </svg>
      </div>
    </Link>
  );
}

export default function ModuleCards(): React.ReactElement {
  return (
    <section className={styles.modulesSection}>
      <div className="container">
        <div className={styles.modulesGrid}>
          {ModuleList.map((module, idx) => (
            <ModuleCard key={idx} {...module} />
          ))}
        </div>
      </div>
    </section>
  );
}