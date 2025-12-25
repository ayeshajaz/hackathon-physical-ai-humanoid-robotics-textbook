# Data Model: Docusaurus UI Upgrade of ai-frontend-book

## UI Components Entity

### Attributes
- **componentType**: String
  - Description: Type of UI component (header, sidebar, content, footer, etc.)
  - Example: "sidebar", "navigation", "content-area"
  - Required: Yes

- **properties**: Object
  - Description: Styling and behavioral properties of the component
  - Example: { color: "#25c2a0", fontSize: "16px", padding: "1rem" }
  - Required: Yes

- **responsiveBehaviors**: Object
  - Description: How the component behaves on different screen sizes
  - Example: { mobile: { display: "none" }, desktop: { display: "block" } }
  - Required: Yes

- **accessibilityFeatures**: Object
  - Description: Accessibility features implemented
  - Example: { ariaLabel: "Main navigation", keyboardNavigation: true }
  - Required: Yes

## Typography System Entity

### Attributes
- **fontFamily**: String
  - Description: Primary font family for content
  - Example: "'Inter', system-ui, sans-serif"
  - Required: Yes

- **fontSizeScale**: Object
  - Description: Scale of different font sizes (headings, body, etc.)
  - Example: { base: "16px", h1: "2.5rem", h2: "2rem", body: "1rem" }
  - Required: Yes

- **lineHeight**: Number
  - Description: Line height ratios for readability
  - Example: 1.7
  - Required: Yes

- **fontWeight**: Number
  - Description: Font weight specifications
  - Example: 400 for body text, 600 for headings
  - Required: Yes

## Responsive Layout Entity

### Attributes
- **breakpoints**: Object
  - Description: Screen size breakpoints for responsive behavior
  - Example: { mobile: "768px", tablet: "1024px", desktop: "1200px" }
  - Required: Yes

- **gridSystem**: Object
  - Description: Grid layout system properties
  - Example: { columns: 12, gutter: "1rem" }
  - Required: Yes

- **spacingScale**: Object
  - Description: Spacing units for consistent design
  - Example: { xs: "0.25rem", sm: "0.5rem", md: "1rem", lg: "1.5rem", xl: "2rem" }
  - Required: Yes

- **mobileBehaviors**: Object
  - Description: How elements adapt on mobile
  - Example: { sidebar: { collapse: true }, navigation: { hamburger: true } }
  - Required: Yes

## Color Palette Entity

### Attributes
- **primary**: String
  - Description: Primary brand color
  - Example: "#25c2a0"
  - Required: Yes

- **secondary**: String
  - Description: Secondary color for accents
  - Example: "#3578e5"
  - Required: Yes

- **background**: String
  - Description: Background color
  - Example: "#ffffff"
  - Required: Yes

- **text**: String
  - Description: Primary text color
  - Example: "#242526"
  - Required: Yes

- **contrastRatios**: Object
  - Description: Contrast ratios for accessibility compliance
  - Example: { textToBackground: 7.5, uiElements: 4.5 }
  - Required: Yes

### Validation Rules
- All UI components must maintain Docusaurus functionality
- Typography must meet readability standards (appropriate line height, font size)
- Color contrast must meet WCAG accessibility guidelines (minimum 4.5:1 ratio)
- Responsive behaviors must work across all specified device sizes
- All existing content must display correctly with new styling