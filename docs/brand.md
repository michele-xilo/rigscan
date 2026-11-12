# Brand

How Rigscan looks and sounds. Use it in the README, the docs, the CLI help, the playground and the viewer.

## Name and tagline

- Name: **Rigscan**, written `rigscan` in monospace when it is the command or the wordmark.
- Tagline: **A health check for robot descriptions.**
- Voice: plain and specific. A finding says what is wrong, where, and what it affects; it does not
  scold. The metaphor is a medical scan: the robot file is the patient, a scan gives a report, and each
  link and joint is healthy, a warning or critical.

## Mascot

A friendly robot inside a scanner frame. A pulse line crosses its visor, a dashed scan line passes behind
it, and three lights on its chest carry the three status colours. It is one drawing in line style: ink
outlines on paper, with the accent used only for the frame, the scan line and the antenna.

| File | Use |
|---|---|
| `assets/logo.svg` | Light background, with its own paper background and rounded corners |
| `assets/logo-dark.svg` | Dark background |
| `assets/logo-transparent.svg` | No background, for a page that already has the paper colour |
| `assets/favicon.svg` | Head only, readable from 16 px |
| `assets/social-preview.png` | 1280 x 640 repository social card |

The logo reads at 160 px. Keep clear space of a quarter of its width around it, and do not recolour,
stretch or add a shadow. In the README use the light and dark files through a `<picture>` element.

## Colour

| Role | Light | Dark | Notes |
|---|---|---|---|
| Paper (background) | `#F3EFE6` | `#14202E` | |
| Ink (text, outlines) | `#14202E` | `#F3EFE6` | 14.3:1 on its background |
| Accent (frame, links, focus) | `#2F5BEA` | `#7F9BFF` | 4.8:1 on light, 6.3:1 on dark |

Status colours are used for health and nothing else. The CLI prints them as the terminal's own teal,
yellow and red (ANSI 36, 33, 31), so they follow the user's theme; the rest of the brand uses the values
below.

| Status | Light fill | Light text | Dark |
|---|---|---|---|
| Healthy | `#1F9D8F` | `#117A6E` | `#3CC4B3` |
| Warning | `#D99A1E` | `#8A5A00` | `#F0B33C` |
| Critical | `#C8372D` | `#C8372D` | `#F26B63` |

The light fills for healthy and warning are below 3:1 on paper, so they are for shapes and badges. Text in
those colours on a light background uses the darker "text" value, which is at least 4.5:1. Never carry
status by colour alone: write the word (`critical`, `warning`, `healthy`) next to it.

The accent is a cool blue so that it never reads as a status.

## Type

No font files are shipped; both stacks are system fonts.

- Text: `ui-sans-serif, system-ui, -apple-system, "Segoe UI", sans-serif`.
- Code, the command and the wordmark: `ui-monospace, "SF Mono", Menlo, Consolas, monospace`.

## Terminal

- Colour follows `--color auto|always|never` and `NO_COLOR`. Without colour the output is the same words.
- Severity is the word plus the colour: `critical` red, `warning` yellow, `healthy` teal.
- Rule codes (`RS001`) are always shown, so a finding can be searched for and ignored by code.
