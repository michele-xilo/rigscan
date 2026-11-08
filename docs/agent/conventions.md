# Conventions

## Workflow

- Write the failing test first, run it, and read the failure: it must fail for
  the intended reason (for a rule: 0 findings where 1 is expected), not for a
  setup mistake. Then implement. Then run the whole suite.
- A new lint rule needs: an ID in `RSnnn` order, a test that triggers it with
  the exact expected span, and a "healthy" case that must not trigger it.
- Keep changes minimal. No refactors or cleanups bundled with a feature.
- Do not commit or push unless the user asks. Do not create branches without
  telling the user.

## Commands

```bash
cargo test
cargo build -p rigscan-core --target wasm32-unknown-unknown
```

`rustup target add wasm32-unknown-unknown` is already installed on the
author's machine.

## Code layout

- `crates/rigscan-core/src/lib.rs`: types (`Span`, `Link`, `Joint`, `Urdf`,
  `Finding`, `Severity`), `parse_urdf`, `lint`. Split into modules when it
  grows; not before.
- `crates/rigscan-core/tests/`: `urdf_parse.rs`, `lint_rules.rs`.
- Span: 1-based line and column, column counted in characters.
- The core must stay WASM-compatible: no filesystem or threads in
  `rigscan-core`. File reading belongs in the CLI crate.

## Style

- Findings use the clinical voice: name the element, state the cause.
  Example: "joint `elbow` child link `ghost` is not defined".
- Colours for the brand: teal = healthy, amber = warning, red = critical. Red
  and amber are reserved for findings.
