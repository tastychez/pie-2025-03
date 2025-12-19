# Project AURA - Vanilla Website

A clean, semantic multi-page website built with vanilla HTML, CSS, and JavaScript for Project AURA (Autonomous Robot Assistant).

## Description

This is a complete website port of the Project AURA React site to vanilla HTML/CSS/JS. It includes all pages from the original site with full content, financial breakdowns, team information, and system diagrams. The site demonstrates best practices for semantic HTML, mobile-first CSS, and progressive enhancement with vanilla JavaScript.

## Preview Locally

### Option 1: Python HTTP Server

```bash
cd vanilla-site
python -m http.server 8000
```

Then open `http://localhost:8000` in your browser.

### Option 2: Using Node.js serve

```bash
cd vanilla-site
npx serve .
```

Follow the instructions to open the URL in your browser.

### Option 3: Using PHP (if installed)

```bash
cd vanilla-site
php -S localhost:8000
```

## Customization

### Content

All pages contain editable content. Key areas:

- **Team members**: Edit team names and roles in `index.html`
- **Financial breakdown**: Update data in `js/financial-data.js`
- **Design decisions**: Add content to placeholders in `design-process.html`
- **Diagrams**: Replace diagram placeholders with actual images in `assets/`

### Styling (css/styles.css)

- **Colors**: Modify CSS custom properties in `:root` at the top of the file
- **Typography**: Update font families and sizes
- **Layout**: Adjust grid/flexbox properties for different layouts
- **Responsive breakpoints**: Change `@media` query breakpoints as needed

### JavaScript

- **main.js**: Handles navigation, mobile menu, smooth scrolling, and active link highlighting
- **financial-data.js**: Contains all financial breakdown data
- **financial-table.js**: Renders the financial breakdown table dynamically

### Assets (assets/)

- `system-diagram.jpg` - System flow diagram
- `data-flow-energy-diagram.jpg` - Power and communication diagram
- Add electrical diagram and other assets as needed

## Structure

```
vanilla-site/
├── README.md                    # This file
├── index.html                   # Home page (Hero, Team, Video, Sections)
├── overview.html                # Project objectives, system diagrams, financial breakdown
├── design-process.html          # Design process overview and decisions
├── mechanical.html              # Mechanical overview
├── electrical.html              # Electrical overview with details
├── software.html                # Software/Firmware overview
├── css/
│   └── styles.css              # All styles (responsive, mobile-first)
├── js/
│   ├── main.js                 # Navigation, mobile menu, smooth scrolling
│   ├── financial-data.js       # Financial breakdown data
│   └── financial-table.js      # Financial table renderer
└── assets/                      # Images and other assets
    ├── system-diagram.jpg      # System flow diagram
    ├── data-flow-energy-diagram.jpg  # Power and communication diagram
    └── placeholder.txt
```

## Pages

- **index.html** - Home page with project introduction, team members, video placeholder, and links to subsections
- **overview.html** - Project objectives, system overview diagrams, and complete financial breakdown
- **design-process.html** - Design process narrative and biggest design decisions
- **mechanical.html** - Mechanical engineering overview
- **electrical.html** - Electrical system details and diagram placeholder
- **software.html** - Software/Firmware system overview

## Browser Support

Works in all modern browsers (Chrome, Firefox, Safari, Edge). No polyfills needed for basic functionality.

## Inspiration

This project structure and approach was inspired by: https://github.com/olincollege/pie-2025-03/blob/main/docs/example-css-page/index.html
