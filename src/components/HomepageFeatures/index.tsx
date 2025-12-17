import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
<<<<<<< HEAD
  Svg?: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
  icon?: string; // For emoji icons
=======
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
>>>>>>> d85aeea8c5d135ba5736162a0277cbb7f9b0a54a
};

const FeatureList: FeatureItem[] = [
  {
<<<<<<< HEAD
    title: 'Physical AI Fundamentals',
    icon: '🧠',
    description: (
      <>
        Understand the core principles of embodied intelligence and how AI systems
        operate in physical environments through sensing, reasoning, and action.
        Explore the fundamentals of real-world interaction and adaptive behavior.
=======
    title: 'Easy to Use',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Docusaurus was designed from the ground up to be easily installed and
        used to get your website up and running quickly.
>>>>>>> d85aeea8c5d135ba5736162a0277cbb7f9b0a54a
      </>
    ),
  },
  {
<<<<<<< HEAD
    title: 'Humanoid Control Systems',
    icon: '⚙️',
    description: (
      <>
        Master the architecture of humanoid robotics control, including motion planning,
        kinematics, dynamics, and real-time control systems that enable natural movement.
        Learn how to implement stable and efficient locomotion patterns.
=======
    title: 'Focus on What Matters',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Docusaurus lets you focus on your docs, and we&apos;ll do the chores. Go
        ahead and move your docs into the <code>docs</code> directory.
>>>>>>> d85aeea8c5d135ba5736162a0277cbb7f9b0a54a
      </>
    ),
  },
  {
<<<<<<< HEAD
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
=======
    title: 'Powered by React',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Extend or customize your website layout by reusing React. Docusaurus can
        be extended while reusing the same header and footer.
>>>>>>> d85aeea8c5d135ba5736162a0277cbb7f9b0a54a
      </>
    ),
  },
];

<<<<<<< HEAD
function Feature({title, Svg, description, icon}: FeatureItem) {
  return (
    <div className={clsx('col', styles.featureCard)}>
      <div className={clsx('text--center', styles.featureIcon)}>
        
        {icon ? (
          <span className={styles.emojiIcon}>{icon}</span>
        ) : Svg ? (
          <Svg className={styles.featureSvg} role="img"/>
        ) : null}
      </div>
      <div className={clsx('padding-horiz--md', styles.featureContent)}>
        <Heading as="h3" className={styles.featureTitle}>{title}</Heading>
        <p className={styles.featureDescription}>{description}</p>
=======
function Feature({title, Svg, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
>>>>>>> d85aeea8c5d135ba5736162a0277cbb7f9b0a54a
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
<<<<<<< HEAD
    <section className={clsx(styles.features, 'padding-vert--lg')}>
=======
    <section className={styles.features}>
>>>>>>> d85aeea8c5d135ba5736162a0277cbb7f9b0a54a
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
<<<<<<< HEAD
}
=======
}
>>>>>>> d85aeea8c5d135ba5736162a0277cbb7f9b0a54a
