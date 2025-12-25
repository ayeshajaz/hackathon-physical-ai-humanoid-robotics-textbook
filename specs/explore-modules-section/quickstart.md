# Quickstart: Explore All Modules Section Implementation

## Prerequisites
- Node.js and npm installed
- Docusaurus project set up and running
- Basic knowledge of React and JSX

## Step 1: Create the ModuleCard Component

Create a new file at `src/components/ModuleCard/index.js`:

```jsx
import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './ModuleCard.module.css';

const ModuleCard = ({ title, description, link }) => {
  return (
    <div className={clsx('col col--4', styles.moduleCard)}>
      <div className={styles.moduleCardInner}>
        <h3 className={styles.title}>{title}</h3>
        <p className={styles.description}>{description}</p>
        <Link to={link} className={styles.button}>
          Open Module →
        </Link>
      </div>
    </div>
  );
};

export default ModuleCard;
```

Create the CSS module at `src/components/ModuleCard/ModuleCard.module.css`:

```css
.moduleCard {
  padding: 1rem;
}

.moduleCardInner {
  padding: 2rem 1rem;
  height: 100%;
  display: flex;
  flex-direction: column;
  /* Clean, academic styling */
  border: 1px solid var(--ifm-color-emphasis-200);
  border-radius: 8px;
  background-color: var(--ifm-card-background-color);
  box-shadow: 0 1px 2px 0 rgba(0, 0, 0, 0.05);
}

.title {
  margin-bottom: 1rem;
  font-size: 1.25rem;
  font-weight: bold;
  color: var(--ifm-heading-color);
}

.description {
  margin-bottom: 1.5rem;
  flex-grow: 1;
  color: var(--ifm-font-color-base);
}

.button {
  display: inline-block;
  padding: 0.5rem 1rem;
  background-color: var(--ifm-color-primary);
  color: white;
  text-decoration: none;
  border-radius: 4px;
  transition: background-color 0.15s ease;
  align-self: flex-start;
}

.button:hover {
  background-color: var(--ifm-color-primary-dark);
  text-decoration: none;
  color: white;
}

@media screen and (max-width: 996px) {
  .moduleCard {
    margin-bottom: 1rem;
  }

  .moduleCardInner {
    padding: 1.5rem;
  }
}

@media screen and (max-width: 768px) {
  .moduleCard {
    flex: 0 0 100%;
    max-width: 100%;
  }
}
```

## Step 2: Create the ModuleSection Component

Create a new file at `src/components/ModuleSection/index.js`:

```jsx
import React from 'react';
import clsx from 'clsx';
import styles from './ModuleSection.module.css';
import ModuleCard from '../ModuleCard';

const moduleData = [
  {
    id: 1,
    title: 'ROS 2 Foundations',
    description: 'Learn ROS 2 — the nervous system of modern robots. Build nodes, topics, services, actions, publishers, subscribers, QoS, and real robot workflows.',
    link: '/docs/ros-foundations', // Adjust path as needed
    order: 1
  },
  {
    id: 2,
    title: 'Simulation & Digital Twins',
    description: 'Master simulation systems: Gazebo, Unity Robotics, Isaac Sim, and digital twin workflows for training and testing robots safely.',
    link: '/docs/simulation-digital-twins', // Adjust path as needed
    order: 2
  },
  {
    id: 3,
    title: 'Hardware Foundations',
    description: 'Motors, actuators, torque control, IMUs, sensors, microcontrollers, and embedded systems required for humanoid robots.',
    link: '/docs/hardware-foundations', // Adjust path as needed
    order: 3
  },
  {
    id: 4,
    title: 'VLA — Vision, Language, Action',
    description: 'Advanced robotics architecture combining perception models, large language models, and action planning systems.',
    link: '/docs/vla-architecture', // Adjust path as needed
    order: 4
  },
  {
    id: 5,
    title: 'Sensor Systems',
    description: 'Depth cameras, LiDAR, sensor fusion, perception pipelines, and environmental awareness for autonomous robots.',
    link: '/docs/sensor-systems', // Adjust path as needed
    order: 5
  },
  {
    id: 6,
    title: 'Locomotion & Control',
    description: 'Humanoid walking, balance, gait planning, motion control, and stability mechanisms.',
    link: '/docs/locomotion-control', // Adjust path as needed
    order: 6
  }
];

const ModuleSection = () => {
  return (
    <section className={clsx('container', styles.moduleSection)}>
      <h2 className={styles.sectionTitle}>Explore All Modules</h2>
      <div className="row">
        {moduleData.map((module) => (
          <ModuleCard
            key={module.id}
            title={module.title}
            description={module.description}
            link={module.link}
          />
        ))}
      </div>
    </section>
  );
};

export default ModuleSection;
```

Create the CSS module at `src/components/ModuleSection/ModuleSection.module.css`:

```css
.moduleSection {
  padding: 4rem 0;
  text-align: center;
}

.sectionTitle {
  margin-bottom: 3rem;
  font-size: 2rem;
  font-weight: bold;
  text-align: center;
  color: var(--ifm-heading-color);
}

/* Minimal spacing between cards */
.moduleSection .row {
  margin: 0 -0.5rem;
}

.moduleSection .col {
  padding: 0 0.5rem;
  margin-bottom: 1rem;
}

@media screen and (max-width: 996px) {
  .moduleSection {
    padding: 3rem 0;
  }

  .sectionTitle {
    font-size: 1.75rem;
    margin-bottom: 2rem;
  }

  .moduleSection .col {
    flex: 0 0 calc(50% - 1rem);
    max-width: calc(50% - 1rem);
    margin-bottom: 1rem;
  }
}

@media screen and (max-width: 768px) {
  .moduleSection {
    padding: 2rem 0;
  }

  .sectionTitle {
    font-size: 1.5rem;
    margin-bottom: 1.5rem;
  }

  .moduleSection .col {
    flex: 0 0 100%;
    max-width: 100%;
  }
}
```

## Step 3: Integrate into Homepage

Update your homepage file at `src/pages/index.js` to include the new component:

```jsx
import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import ModuleSection from '../components/ModuleSection';
// ... other imports

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Learning →
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        <ModuleSection />
        {/* Add other sections as needed */}
      </main>
    </Layout>
  );
}
```

## Step 4: Update Documentation Paths

Update the link paths in the `moduleData` array to match your actual Docusaurus documentation structure.

## Step 5: Test Responsiveness

1. Test the layout on different screen sizes
2. Adjust CSS as needed to ensure proper responsive behavior
3. Verify accessibility features work correctly

## Running the Project

After implementing the changes:

```bash
npm start
```

The new "Explore All Modules" section should appear on your homepage below the hero section.