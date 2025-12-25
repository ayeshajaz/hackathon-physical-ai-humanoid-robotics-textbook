# Docusaurus Module Section Component

## Overview
The Module Section component displays six module cards in a responsive grid layout below the hero section on the homepage. Each card contains a module title, academic description, and navigation button to the corresponding documentation page. This component helps users discover all available modules in the Physical AI & Humanoid Robotics textbook.

## Component Structure
```
src/
└── components/
    ├── ModuleCard/
    │   ├── index.js
    │   └── ModuleCard.module.css
    └── ModuleSection/
        ├── index.js
        └── ModuleSection.module.css
```

## Usage
The component is integrated into the homepage at `ai-frontend-book/src/pages/index.js` and displays six module cards with the following content:

1. **ROS 2 Foundations**: Information about ROS 2 fundamentals
2. **Simulation & Digital Twins**: Information about simulation systems
3. **Hardware Foundations**: Information about hardware components
4. **VLA — Vision, Language, Action**: Information about VLA architecture
5. **Sensor Systems**: Information about sensor technologies
6. **Locomotion & Control**: Information about movement and control systems

## Features
- Responsive design (3 cards on desktop, 2 on tablet, 1 on mobile)
- Clean, academic styling with proper typography
- Accessibility support (ARIA labels, keyboard navigation)
- Overflow handling for long titles/descriptions
- Link fallbacks for invalid navigation paths
- Proper heading hierarchy (H2 for section, H3 for modules)