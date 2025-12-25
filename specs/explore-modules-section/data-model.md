# Data Model: Explore All Modules Section

## Entity: ModuleCard

### Fields
- **id** (string): Unique identifier for the module
- **title** (string): Module title text
- **description** (string): Academic description text
- **link** (string): URL path to the module documentation
- **order** (number): Display order for the card (1-6)

### Validation Rules
- **title**: Required, maximum 100 characters for readability
- **description**: Required, maximum 300 characters to maintain academic brevity
- **link**: Required, must be a valid Docusaurus documentation path
- **order**: Required, must be between 1 and 6, unique across all cards

### Relationships
- Belongs to: ModuleSection (one section contains multiple module cards)
- Dependencies: Docusaurus documentation paths

## Entity: ModuleSection

### Fields
- **title** (string): Section title ("Explore All Modules")
- **cards** (array of ModuleCard): The collection of six module cards
- **layout** (string): Layout configuration (e.g., "responsive-grid", "mobile-column")

### Validation Rules
- **title**: Required, must be "Explore All Modules"
- **cards**: Must contain exactly 6 cards
- **cards.order**: Must have unique values 1 through 6
- **layout**: Must be one of the predefined layout types

### State Transitions
- Initial state: Cards defined in configuration
- Ready state: Cards rendered and visible on homepage
- Responsive state: Layout adjusted based on screen size

## Constraints
- All cards must be displayed simultaneously on the section
- Cards must maintain grid alignment on desktop (3 per row)
- Cards must be responsive and reflow on smaller screens (2 per row on tablet, 1 per row on mobile)
- Cards must maintain accessibility standards
- Section title must be centered with large, bold, academic typography