# Docusaurus Homepage Info Cards Component

## Overview
The Homepage Info Cards component displays three horizontally aligned info cards below the hero section on the homepage. Each card contains an icon, bold title, and crisp description to communicate the key value propositions of the Physical AI & Humanoid Robotics textbook.

## Component Structure
```
src/
└── components/
    ├── Icons/
    │   ├── BookIcon.js
    │   ├── RobotIcon.js
    │   └── GearIcon.js
    ├── InfoCard/
    │   ├── index.js
    │   └── InfoCard.module.css
    └── HomepageInfoCards/
        ├── index.js
        └── HomepageInfoCards.module.css
```

## Usage
The component is integrated into the homepage at `ai-frontend-book/src/pages/index.js` and displays three info cards with the following content:

1. **Comprehensive Coverage**: Information about the textbook's coverage
2. **AI-Powered Q&A**: Information about the Q&A system
3. **Practical Focus**: Information about practical applications

## Features
- Responsive design (3 cards on desktop, 2 on tablet, 1 on mobile)
- Clean, minimal styling with no boxed borders
- Professional academic visual presentation
- Accessibility support (ARIA labels, keyboard navigation)
- Edge case handling for long titles/descriptions
- SVG icon fallbacks