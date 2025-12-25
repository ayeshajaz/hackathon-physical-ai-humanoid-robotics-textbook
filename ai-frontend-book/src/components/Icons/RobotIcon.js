import React from 'react';

const RobotIcon = () => (
  <svg
    xmlns="http://www.w3.org/2000/svg"
    width="48"
    height="48"
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    strokeWidth="2"
    strokeLinecap="round"
    strokeLinejoin="round"
    className="robot-icon"
  >
    <rect x="3" y="11" width="18" height="10" rx="2" />
    <path d="M7 11V7a5 5 0 0 1 10 0v4" />
    <circle cx="15" cy="19" r="2" />
    <circle cx="9" cy="19" r="2" />
    <path d="M12 11v-2" />
    <path d="M16 7h.01" />
    <path d="M8 7h.01" />
  </svg>
);

export default RobotIcon;