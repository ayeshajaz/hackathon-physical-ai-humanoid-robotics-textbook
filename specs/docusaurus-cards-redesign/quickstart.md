# Quickstart: Docusaurus Homepage Info Cards Implementation

## Prerequisites
- Node.js and npm installed
- Docusaurus project set up and running
- Basic knowledge of React and JSX

## Step 1: Create the InfoCard Component

Create a new file at `src/components/InfoCard/index.js`:

```jsx
import React from 'react';
import clsx from 'clsx';
import styles from './InfoCard.module.css';

const InfoCard = ({ icon, title, description }) => {
  return (
    <div className={clsx('col col--4', styles.infoCard)}>
      <div className={styles.infoCardInner}>
        <div className={styles.icon}>{icon}</div>
        <h3 className={styles.title}>{title}</h3>
        <p className={styles.description}>{description}</p>
      </div>
    </div>
  );
};

export default InfoCard;
```

Create the CSS module at `src/components/InfoCard/InfoCard.module.css`:

```css
.infoCard {
  padding: 1rem;
}

.infoCardInner {
  padding: 2rem 1rem;
  text-align: center;
  border-radius: 8px;
  height: 100%;
  display: flex;
  flex-direction: column;
  align-items: center;
}

.icon {
  margin-bottom: 1rem;
  font-size: 2rem;
}

.title {
  margin-bottom: 1rem;
  font-size: 1.25rem;
  font-weight: bold;
}

.description {
  margin-bottom: 0;
  font-size: 1rem;
  line-height: 1.5;
}
```

## Step 2: Create the HomepageInfoCards Component

Create a new file at `src/components/HomepageInfoCards/index.js`:

```jsx
import React from 'react';
import clsx from 'clsx';
import styles from './HomepageInfoCards.module.css';
import InfoCard from '../InfoCard';

const cardData = [
  {
    id: 1,
    icon: '📚', // Replace with actual icon component
    title: 'Comprehensive Coverage',
    description: 'A structured textbook covering Physical AI, ROS 2, simulation, digital twins, perception, and humanoid robotics pipelines.'
  },
  {
    id: 2,
    icon: '🤖', // Replace with actual icon component
    title: 'AI-Powered Q&A',
    description: 'Interactive question-answering grounded in textbook content, enabling precise and hallucination-free learning support.'
  },
  {
    id: 3,
    icon: '⚙️', // Replace with actual icon component
    title: 'Practical Focus',
    description: 'Each chapter emphasizes real-world architectures, algorithms, and deployment-ready considerations for embodied AI systems.'
  }
];

const HomepageInfoCards = () => {
  return (
    <section className={clsx('container', styles.infoCards)}>
      <div className="row">
        {cardData.map((card) => (
          <InfoCard
            key={card.id}
            icon={card.icon}
            title={card.title}
            description={card.description}
          />
        ))}
      </div>
    </section>
  );
};

export default HomepageInfoCards;
```

Create the CSS module at `src/components/HomepageInfoCards/HomepageInfoCards.module.css`:

```css
.infoCards {
  padding: 4rem 0;
}

@media screen and (max-width: 996px) {
  .infoCards {
    padding: 2rem 0;
  }
}
```

## Step 3: Integrate into Homepage

Update your homepage file at `src/pages/index.js` to include the new component:

```jsx
import React from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import HomepageInfoCards from '../components/HomepageInfoCards';
// ... other imports

export default function Home() {
  // ... existing code

  return (
    <Layout
      title={`Hello`}
      description="Description goes here">
      <HomepageHeader />
      <main>
        {/* Add the info cards component here */}
        <HomepageInfoCards />
        {/* Add other sections as needed */}
      </main>
    </Layout>
  );
}
```

## Step 4: Customize Icons

Replace the placeholder icons with actual SVG icons:

1. Create SVG icon components in a new directory `src/components/Icons/`
2. Update the `cardData` array to use your custom icon components
3. Make sure icons are properly sized and styled

## Step 5: Test Responsiveness

1. Test the layout on different screen sizes
2. Adjust CSS as needed to ensure proper responsive behavior
3. Verify accessibility features work correctly

## Running the Project

After implementing the changes:

```bash
npm start
```

The new info cards should appear on your homepage below the hero section.