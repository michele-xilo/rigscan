# Style guide

How code, messages and docs in Rigscan are written. Tooling enforces the
mechanical parts; this file covers the rest. When in doubt, match the code
around you.

## Rust

- Format with `cargo fmt`. CI runs `cargo fmt --all --check`.
- Clippy must be clean: `cargo clippy --workspace --all-targets -- -D warnings`.
- Keep functions short and focused. Prefer a plain loop over a clever iterator
  chain when it reads better.
- Name things after what they are in robotics: `Link`, `Joint`, `Robot`. Use
  the format's own word only inside that format's parser.
- No `unwrap` or `expect` on data that comes from a file. Return an error, or
  report a finding. `unwrap` is fine in tests.
- Never swallow an error. Propagate it, or recover and say so.
- Add a dependency only when the work needs it, and say why in the PR.
- No `unsafe` without a benchmark that shows it is needed and a comment stating
  the invariant it relies on.

### Comments and docs in code

- Comments explain **why**, not what or how. Good: a format quirk, a measured
  decision, a surprising choice. Bad: a sentence that repeats the next line.
- Do not cite issues or PRs in comments.
- Public items get a short doc comment. Private helpers usually do not.
- No emojis in code or comments.

### Layout

- In a file with an inline `#[cfg(test)]` module, all runtime code comes first
  and the test module is last. Clippy enforces that nothing follows it.
- Integration tests live in `crates/<crate>/tests/`. Benchmarks live in
  `crates/<crate>/benches/`.
- `rigscan-core` does no file, network or thread work and must keep building
  for `wasm32-unknown-unknown`. File access belongs in the CLI.

## Tests

- Name tests after the behaviour: `transmission_joint_reference_is_not_a_joint`,
  not `test_parse_2`.
- Parsers take `&str`, not paths, so tests can use inline documents.
- Keep inline documents minimal: only the elements the test is about.
- Compare floats with a tolerance.
- A corpus test that cannot find its data must skip and say why. It must not
  pass silently.

## Lint rules and findings

- Rule IDs are `RS` plus three digits (`RS001`) and are **stable**: never reuse
  or renumber one.
- Every finding carries a source span. Spans are 1-based, and columns count
  characters, not bytes.
- Messages read like a good compiler error: name the element, state the cause,
  and say nothing else. Use backticks around names.
  - Good: ``joint `elbow` child link `ghost` is not defined``
  - Bad: `Error! Something is wrong with your joint.`
- Severity is `critical` (the file is broken or will not load) or `warning`
  (suspicious but usable). Do not add a numeric score.
- Each rule has a test that triggers it with the exact expected span, and a
  healthy case that must not trigger it.

## Command line output

- One finding per line: `path:line:col: severity RULE: message`.
- Findings go to stdout; errors, warnings and the summary line go to stderr.
- Exit codes: `0` clean, `1` critical findings, `2` a file could not be scanned.
- Treat output as an interface. Change it additively.

## Performance

Speed is a feature. See [CONTRIBUTING.md](CONTRIBUTING.md#performance) for the
workflow. In short: measure first, avoid quadratic loops, borrow instead of
cloning, keep output deterministic when work runs in parallel.

## Commit messages

- A short imperative summary prefixed with the area: `core:`, `cli:`, `urdf:`,
  `mjcf:`, `usd:`, `convert:`, `docs:`, `ci:`.
- The body explains *why* when it is not obvious.
- One logical change per commit.

## Writing docs

- Plain and direct. Say what it does and how to run it.
- State the project's actual status. Do not describe features that do not exist
  yet as if they did; put them under "Planned".
- Keep the README and the relevant docs up to date in the same change that
  alters behaviour.
- No emojis in docs, except the repository description.

## Brand

For anything user-facing (CLI colour, viewer, playground): teal means healthy,
amber means warning, red means critical. Red and amber are reserved for
findings. The voice is clinical and calm.
