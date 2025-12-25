import React from 'react';
import clsx from 'clsx';
import styles from './InfoCard.module.css';

const InfoCard = ({ icon, title, description }) => {
  // Create a unique ID based on the title for accessibility
  const titleId = `info-card-title-${title.replace(/\s+/g, '-').toLowerCase()}`;

  return (
    <div
      className={clsx('col col--4', styles.infoCard)}
      role="region"
      aria-labelledby={titleId}
      tabIndex="0"
    >
      <div className={styles.infoCardInner}>
        <div className={styles.icon} aria-hidden="true">
          {icon || <span aria-label="icon">📄</span>} {/* Fallback icon if none provided */}
        </div>
        <h3
          id={titleId}
          className={styles.title}
        >
          {title}
        </h3>
        <p className={styles.description}>{description}</p>
      </div>
    </div>
  );
};

export default InfoCard;