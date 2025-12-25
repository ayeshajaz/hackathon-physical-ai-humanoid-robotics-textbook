import React from 'react';
import clsx from 'clsx';
import styles from './HomepageInfoCards.module.css';
import InfoCard from '../InfoCard';
import BookIcon from '../Icons/BookIcon';
import RobotIcon from '../Icons/RobotIcon';
import GearIcon from '../Icons/GearIcon';

const cardData = [
  {
    id: 1,
    icon: <BookIcon />,
    title: 'Comprehensive Coverage',
    description: 'A structured textbook covering Physical AI, ROS 2, simulation, digital twins, perception, and humanoid robotics pipelines.'
  },
  {
    id: 2,
    icon: <RobotIcon />,
    title: 'AI-Powered Q&A',
    description: 'Interactive question-answering grounded in textbook content, enabling precise and hallucination-free learning support.'
  },
  {
    id: 3,
    icon: <GearIcon />,
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