---
title: Rigscan
emoji: 🦾
colorFrom: gray
colorTo: blue
sdk: gradio
sdk_version: 5.50.0
app_file: app.py
pinned: false
license: apache-2.0
short_description: Health check and conversion for URDF, MJCF and USD robots
---

Rigscan scans URDF, MJCF and USD robot descriptions for broken links, inverted limits and missing
inertia, and converts between the three formats with a report of what each conversion loses.

Source, rules and docs: https://github.com/michele-xilo/rigscan

This Space is built from a tagged release; the `rigscan` binary next to `app.py` is the release's CLI.
