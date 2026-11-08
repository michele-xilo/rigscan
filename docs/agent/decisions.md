# Decisions

Format: decision, then why, then when to revisit.

## D1. Name is Rigscan
Chosen by the user over Roboscanner and Kinwright. `rigscan` was free on
crates.io, PyPI and as a GitHub username on 2026-11-08. Trademarks and domains
were NOT checked. Revisit if a trademark or domain conflict appears.

## D2. Own URDF parser on quick-xml, not `urdf-rs`
`urdf-rs` 0.10.0 builds for wasm32, but it drops source spans and unknown
tags, and a linter needs both. Cost estimated at about a week. The user did not
object when this was proposed. Revisit only if maintaining the parser costs
more than expected.

## D3. Wedge is linting plus URDF <-> MJCF with a loss report; not USD
Newton/NVIDIA ship USD converters weekly. USD is a later, write-only
`.usda`/UsdPhysics target. See context.md for the evidence.

### D3 update (2026-11-08)
The user wants all three formats (URDF, MJCF, USD) validated first, then
conversion between all three. USD is therefore in scope for reading and
validation, not only as a later write-only target. Mitigation: a spike
(issue #14) decides whether the `openusd` crate is usable; fallback is
`.usda` text only.

## D4. Rules are numbered `RSnnn` and stable
IDs are never reused or renumbered. Severities so far: warning, critical. No
single numeric health score: teams would argue about what a 94 means. Report
per-link/per-joint status and a verdict like "14 healthy, 2 warnings, 1
critical".

## D5. Lint findings carry a source span
Every finding points at a line and column. This is the reason for D2.

## D6. Playground uses a 2D tree diagram, not a 3D viewer
A 3D viewer would add weeks; the coloured tree already gives the "scan" feel.

## D7. Xacro is out of v0.1
Users run xacro first and feed the expanded URDF. Native xacro is v0.3.

## D8. Licence is Apache-2.0
Set as a placeholder when the workspace was created, then confirmed by adding
the LICENSE file. The user has not objected. No copyright holder line yet.

## D9. Git workflow: small stacked branches, commit only when asked
Each change goes on its own branch (`feat/urdf-parser`,
`feat/rs001-missing-link`, ...). Branches are currently stacked and not merged
into `main`; nothing is pushed. Do not commit unless the user asks.

## D10. Do not add dependencies speculatively
`quick-xml` 0.42 is the only dependency. In 0.42 element names are `&str`
(not bytes) and `Attribute::unescape_value` is deprecated in favour of
`normalized_value(XmlVersion::Implicit1_0)`.

## D11. Work is tracked as GitHub issues
Repo: https://github.com/michele-xilo/rigscan (public, created by the user;
visibility confirmed public by the user). 27 issues, numbered in plan order,
labelled by priority (P0/P1/P2), size (S/M/L) and phase. Dependencies are
listed in each issue body. No code has been pushed yet.

## Open questions (ask the user, do not assume)

- Who is building this and how many hours a week? Decides whether the 14-week
  plan holds.
- Remote repo, GitHub org, domain, CI.
- Whether the CLI binary is `rigscan` only or also a short alias.
