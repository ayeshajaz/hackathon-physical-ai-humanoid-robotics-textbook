import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './ModuleCard.module.css';

const ModuleCard = ({ title, description, link }) => {
  // Create a unique ID based on the title for accessibility
  const titleId = `module-card-title-${title.replace(/\s+/g, '-').toLowerCase()}`;

  return (
    <div
      className={clsx('col col--4', styles.moduleCard)}
      role="region"
      aria-labelledby={titleId}
    >
      <div className={styles.moduleCardInner} tabIndex="0">
        <h3
          id={titleId}
          className={styles.title}
        >
          {title}
        </h3>
        <p className={styles.description}>{description}</p>
        <Link
          to={link || '#'}
          className={styles.button}
          onClick={(e) => {
            if (!link) {
              e.preventDefault();
              console.warn(`Invalid link for module: ${title}`);
            }
          }}
        >
          Open Module →
        </Link>
      </div>
    </div>
  );
};

export default ModuleCard;