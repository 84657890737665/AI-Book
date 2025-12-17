import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg?: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
  icon?: string; // For emoji icons
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Physical AI Fundamentals',
    icon: '🧠',
    description: (
      <>
        Understand the core principles of embodied intelligence and how AI systems
        operate in physical environments through sensing, reasoning, and action.
        Explore the fundamentals of real-world interaction and adaptive behavior.
      </>
    ),
  },
  {
    title: 'Humanoid Control Systems',
    icon: '⚙️',
    description: (
      <>
        Master the architecture of humanoid robotics control, including motion planning,
        kinematics, dynamics, and real-time control systems that enable natural movement.
        Learn how to implement stable and efficient locomotion patterns.
      </>
    ),
  },
  {
    title: 'Sensors & Actuators',
    icon: '📡',
    description: (
      <>
        Discover the critical components that enable robots to perceive and interact with
        their environment. From LiDAR to cameras and tactile sensors to precise actuators,
        understand how hardware enables intelligent behavior.
      </>
    ),
  },
  {
    title: 'Learning in the Real World',
    icon: '🎓',
    description: (
      <>
        Explore reinforcement learning, imitation learning, and other techniques that
        allow robots to learn from physical interaction with their environment.
        Master transfer learning from simulation to real-world deployment.
      </>
    ),
  },
  {
    title: 'Human-Robot Interaction',
    icon: '🤝',
    description: (
      <>
        Understand the principles of safe and intuitive interaction between humans
        and robots. Explore communication protocols, ethical considerations, and
        social robotics research that makes robots collaborative partners.
      </>
    ),
  },
  {
    title: 'Ethics & Future of Robotics',
    icon: '⚖️',
    description: (
      <>
        Examine the ethical implications of humanoid robots in society and the
        future trajectory of the field. Address safety, privacy, and societal
        impact considerations for responsible deployment of embodied AI.
      </>
    ),
  },
];

function Feature({title, Svg, description, icon}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        {icon ? (
          <span className={styles.emojiIcon}>{icon}</span>
        ) : Svg ? (
          <Svg className={styles.featureSvg} role="img" />
        ) : null}
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
