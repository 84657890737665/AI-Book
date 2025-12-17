import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';
import Chatbot from '@site/src/components/Chatbot/Chatbot';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx(styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroText}>
            <Heading as="h1" className={styles.heroTitle}>
              Physical AI and Humanoid Robotics
            </Heading>
            <p className={styles.heroSubtitle}>
              Intelligence That Lives, Moves, and Learns in the Physical World
            </p>
            <div className={styles.heroButtons}>
              <Link
                className={clsx('button button--primary button--lg', styles.heroButton)}
                to="/docs/intro">
                Read Book
              </Link>
              <Link
                className={clsx('button button--secondary button--lg', styles.heroButton)}
                to="/docs/textbook-overview">
                Explore Modules
              </Link>
            </div>
          </div>
          <div className={styles.heroVisual}>
            <img
              src="/img/humonoid-hero.png"
              alt="Humanoid Robot Visualization"
              className={styles.robotImage}
            />
          </div>
        </div>
      </div>
    </header>
  );
}

function BookOverview() {
  return (
    <section className={styles.bookOverview}>
      <div className="container">
        <div className="row">
          <div className="col col--6">
            <div className={styles.holographicPanel}>
              <Heading as="h2" className={styles.sectionTitle}>Physical Intelligence & Embodied AI</Heading>
              <p className={styles.sectionDescription}>
                Discover how AI systems can perceive, reason, and act in the physical world through embodied cognition.
                Learn about the principles that govern intelligence emerging from the interaction between mind and body.
              </p>
            </div>
            <div className={styles.holographicPanel}>
              <Heading as="h2" className={styles.sectionTitle}>Humanoid Motion and Perception</Heading>
              <p className={styles.sectionDescription}>
                Explore advanced techniques for enabling humanoid robots to move naturally and perceive their environment
                using multi-modal sensory systems that mirror human capabilities.
              </p>
            </div>
          </div>
          <div className="col col--6">
            <div className={styles.holographicPanel}>
              <Heading as="h2" className={styles.sectionTitle}>AI-Driven Robotics Systems</Heading>
              <p className={styles.sectionDescription}>
                Master the integration of artificial intelligence with robotic systems to create autonomous agents
                capable of complex real-world tasks and interactions.
              </p>
            </div>
            <div className={styles.holographicPanel}>
              <Heading as="h2" className={styles.sectionTitle}>Human-Robot Interaction</Heading>
              <p className={styles.sectionDescription}>
                Understand the principles of intuitive and safe interaction between humans and robots,
                including communication protocols and collaborative workflows.
              </p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Physical AI & Humanoid Robotics`}
      description="Comprehensive guide to building intelligent systems that operate in physical space">
      <HomepageHeader />
      <main>
        <BookOverview />
        <HomepageFeatures />
      </main>
      <Chatbot />
    </Layout>
  );
}