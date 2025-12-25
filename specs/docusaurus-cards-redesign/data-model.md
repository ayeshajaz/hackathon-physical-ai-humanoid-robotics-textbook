# Data Model: Docusaurus Homepage Info Cards

## Entity: HomepageInfoCard

### Fields
- **id** (string): Unique identifier for the card
- **icon** (string or React component): Icon element to display at the top of the card
- **title** (string): Bold title text for the card
- **description** (string): Short, crisp description text
- **order** (number): Display order for the card (1-3)

### Validation Rules
- **title**: Required, maximum 50 characters for readability
- **description**: Required, maximum 200 characters to maintain crispness
- **icon**: Required, must be a valid React component or path to SVG asset
- **order**: Required, must be between 1 and 3, unique across all cards

### Relationships
- Belongs to: Homepage (one homepage contains multiple info cards)
- Dependencies: Icon assets, styling components

## Entity: InfoCardsCollection

### Fields
- **cards** (array of HomepageInfoCard): The collection of three info cards
- **layout** (string): Layout configuration (e.g., "horizontal-grid", "responsive-flex")

### Validation Rules
- **cards**: Must contain exactly 3 cards
- **cards.order**: Must have unique values 1, 2, and 3
- **layout**: Must be one of the predefined layout types

### State Transitions
- Initial state: Cards defined in configuration
- Ready state: Cards rendered and visible on homepage
- Responsive state: Layout adjusted based on screen size

## Constraints
- All cards must be displayed simultaneously on the homepage
- Cards must maintain horizontal alignment on desktop
- Cards must be responsive and reflow on smaller screens
- Cards must maintain accessibility standards