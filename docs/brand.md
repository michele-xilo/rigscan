# Brand

How Rigscan looks and sounds. Use it in the README, the docs, the CLI help, the playground and the viewer.

## Name and tagline

- Name: **Rigscan**, written `rigscan` in monospace when it is the command or the wordmark.
- Tagline: **A health check for robot descriptions.**
- Voice: plain and specific. A finding says what is wrong, where, and what it affects; it does not
  scold. The metaphor is a medical scan: the robot file is the patient, a scan gives a report, and each
  link and joint is healthy, a warning or critical.

## Logo

A scanner frame (four corner brackets) around a kinematic chain: three joints joined by two links. It is
drawn in line style, white on black, with no other colour. The wordmark is `rigscan` in bold monospace.

| File | Use |
|---|---|
| `assets/logo.svg` | Mark and wordmark on black, with rounded corners. The README header |
| `assets/mark.svg` | The mark alone, white strokes on a transparent background, for a dark page |
| `assets/favicon.svg` | The mark on a black rounded square, readable from 16 px |
| `assets/social-preview.png` | 1280 x 640 repository social card |

The mark reads at 16 px. Keep clear space of a quarter of its width around it, and do not recolour,
stretch, outline or add a shadow. The identity is dark only: place the mark on black or near-black.

## Colour

| Role | Value | Contrast on `#0B0B0C` |
|---|---|---|
| Background | `#0B0B0C` | |
| Panel | `#131316` | |
| Line (borders) | `#26262B` | |
| Text | `#EDEDF0` | 16.8:1 |
| Muted text | `#A1A1AA` | 7.7:1 |
| Accent (focus, selection, links) | `#8B8BFF` | 6.8:1 |

Status colours are used for health and nothing else. The CLI prints them as the terminal's own teal,
yellow and red (ANSI 36, 33, 31), so they follow the user's theme; the rest of the brand uses the values
below.

| Status | Value | Contrast on `#0B0B0C` |
|---|---|---|
| Healthy | `#3CC4B3` | 9.1:1 |
| Warning | `#F0B33C` | 10.5:1 |
| Critical | `#F26B63` | 6.6:1 |

Never carry status by colour alone: write the word (`critical`, `warning`, `healthy`) next to it. The
accent is a cool violet so that it never reads as a status.

## Type

No font files are shipped; both stacks are system fonts.

- Text: `ui-sans-serif, system-ui, -apple-system, "Segoe UI", sans-serif`.
- Code, the command and the wordmark: `ui-monospace, "SF Mono", Menlo, Consolas, monospace`.

## Terminal

- Colour follows `--color auto|always|never` and `NO_COLOR`. Without colour the output is the same words.
- Severity is the word plus the colour: `critical` red, `warning` yellow, `healthy` teal.
- Rule codes (`RS001`) are always shown, so a finding can be searched for and ignored by code.
