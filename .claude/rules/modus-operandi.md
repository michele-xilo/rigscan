# Modus Operandi

## What this project is

Rigscan is a health check for robot descriptions: it validates URDF, MJCF and USD files, and converts
between them while reporting what each conversion loses. The core (`crates/rigscan-core`) is Rust and
must stay WebAssembly-compatible; the CLI, Python bindings and browser playground are thin layers on
top of it. Work is tracked as GitHub issues in `michele-xilo/rigscan`; agent-facing context and decisions
are in `docs/agent/`.

## Build, lint, format, test: do this often

Run these frequently during development, not only at the end.

```bash
cargo fmt --all
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
cargo build -p rigscan-core --target wasm32-unknown-unknown
```

Performance is a requirement: see `performance.md` for benchmarking (`cargo bench`) and parallelism.

Run all the checks before considering work done. Passing tests verify the code, not real robots: a change
to a parser or a rule must also run against the corpus (see Testing). If you cannot run something, say so
rather than claim it works.

## Code style

- Comments explain **why**, never what or how. A useful comment captures something the reader can't
  recover from the code: a format quirk, a measured decision, a counter-intuitive choice. Strip comments
  that paraphrase the code. Comments do not cite issues or PRs.
- Doc comments on public items are fine; don't add them to every private function.
- No emojis in code or comments.
- Keep functions short and focused.
- Never let errors pass silently. Return an error, or emit a finding with context. No `unwrap`/`expect`
  on input from files in library code.
- Rust files: all runtime code first, then the `#[cfg(test)]` module at the end, never code-test-code-test.
  Clippy's `items_after_test_module` enforces that no item follows a test module.
- `rigscan-core` does no filesystem, network or thread work. File access goes through a resolver passed in
  by the caller (the CLI supplies a real one).
- Add dependencies only when a ticket needs them.

## Contracts

- Rule IDs (`RSnnn`) are stable: never reuse or renumber one. Retire a rule by documenting it, not by
  recycling its ID.
- JSON and SARIF output are contracts with users and CI. Change them additively; bump a version field for
  anything else.
- Every finding carries a source span (line and column, 1-based).

## Testing

- Every rule has a test that triggers it with the exact expected span, and a healthy case that must not
  trigger it.
- Put logic where it can be tested without files, the network or a GPU. Parsers take `&str`, not paths.
- Match the existing style in `crates/rigscan-core/tests/`: plain `#[test]` functions with names that state
  the behaviour. Compare floats with a tolerance.
- Corpus tests (real URDF, MJCF and USD robots) must skip with a stated reason when the corpus is not
  present, never silently pass. Do not commit large or differently licensed robot assets; fetch them in CI
  or keep small hand-written fixtures.

## Results and reproducibility

- Every reported number (false-positive rate, files parsed, speed) names the corpus and its version or commit,
  and the Rigscan commit it was measured on.

## Specs, decisions, notes

- Design and decisions for a change are written on its GitHub issue (the approved plan as a comment, later
  decisions as further comments).
- `docs/` holds what users need: how to run it, formats, rules. `docs/agent/` holds present-tense context
  and conventions for agents. Decisions, plans and status are not committed: see `issue-tracking.md`.

## Data

- Never commit credentials, `.env`, tokens, or large third-party robot assets.

## README

Keep `README.md` (and `docs/` where they describe the changed behaviour) up to date in the same change when
you add a command, option, output format or rule. A new rule also needs its probe in `scripts/compare/probes/`
and a rerun of `scripts/compare/compare.py` (`--bench` when the core changes speed).

## Git

- Branch from `main` for feature work. Do not commit or push unless the user asks.
- Commit messages: short imperative summary prefixed with the area (`core: ...`, `cli: ...`, `urdf: ...`,
  `mjcf: ...`, `usd: ...`, `convert: ...`, `docs: ...`, `ci: ...`). Body explains *why* if non-obvious.
  End with the Co-Authored-By trailer when an agent wrote the change.
